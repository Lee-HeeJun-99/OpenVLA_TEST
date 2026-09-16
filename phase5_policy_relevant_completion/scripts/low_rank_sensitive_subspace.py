#!/usr/bin/env python3
"""LOO low-rank policy-sensitive subspace correction."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
LHJ = ROOT / "lhj"
BASE_SCRIPT_DIR = LHJ / "scripts"
if str(BASE_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_SCRIPT_DIR))

from action_hidden_sensitive_null_decomposition import grad_direction, project_onto  # noqa: E402
from action_hidden_token_sensitivity import (  # noqa: E402
    ANALYSIS_ROOT,
    CHECKPOINT,
    DATASET_KEY,
    FEATURE,
    PHASE_CSV,
    action_chunk,
    chunk_metrics,
    episode_id_from_frame,
    load_action_head,
    load_npz_array,
    phase_map,
    predict_batch,
    progress_from_frame,
    read_json,
    records_by_frame,
    write_csv,
    write_json,
)


PHASE5 = LHJ / "phase5_policy_relevant_completion"
OUTPUT_DIR = PHASE5 / "03_layerwise_tracing"
LOG_PATH = LHJ / "작업기록.md"
K_VALUES = [1, 2, 4, 8, 16, 32, 64, 128]
SEED = 20260916


def flatten(arr: np.ndarray) -> np.ndarray:
    return arr.reshape(-1).astype(np.float64)


def build_basis(vectors: list[np.ndarray], k_max: int) -> np.ndarray:
    x = np.stack([flatten(v) for v in vectors], axis=0)
    x = x - x.mean(axis=0, keepdims=True)
    gram = x @ x.T
    vals, vecs = np.linalg.eigh(gram)
    order = np.argsort(vals)[::-1]
    vals = vals[order]
    vecs = vecs[:, order]
    keep = vals > 1e-10
    vals = vals[keep]
    vecs = vecs[:, keep]
    k = min(k_max, vals.size)
    if k == 0:
        return np.zeros((0, x.shape[1]), dtype=np.float64)
    basis = (vecs[:, :k].T @ x) / np.sqrt(vals[:k, None])
    basis /= np.maximum(np.linalg.norm(basis, axis=1, keepdims=True), 1e-12)
    return basis


def project_with_basis(delta: np.ndarray, basis: np.ndarray, shape: tuple[int, ...], k: int) -> np.ndarray:
    if k <= 0 or basis.shape[0] == 0:
        return np.zeros(shape, dtype=np.float64)
    b = basis[: min(k, basis.shape[0])]
    flat = flatten(delta)
    coeff = b @ flat
    return (coeff @ b).reshape(shape)


def random_basis(dim: int, k: int, rng: np.random.Generator) -> np.ndarray:
    mat = rng.standard_normal((k, dim))
    q, _ = np.linalg.qr(mat.T)
    return q.T.astype(np.float64)


def aggregate(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[k] for k in group_keys), []).append(row)
    metrics = [
        "gap_to_sim_chunk_mean_l2",
        "gap_to_sim_first_translation_l2",
        "gap_to_sim_first_rotation_l2",
        "gap_to_sim_first_gripper_abs",
        "repr_gap_reduction_ratio",
        "action_gap_reduction",
    ]
    out = []
    for key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        rec = {k: v for k, v in zip(group_keys, key)}
        rec["count"] = len(items)
        for metric in metrics:
            vals = np.asarray([float(item[metric]) for item in items], dtype=np.float64)
            rec[f"{metric}_mean"] = float(vals.mean())
            rec[f"{metric}_p50"] = float(np.percentile(vals, 50))
            rec[f"{metric}_p90"] = float(np.percentile(vals, 90))
        rec["improved_count"] = int(sum(float(item["action_gap_reduction"]) > 0 for item in items))
        rec["worsened_count"] = int(sum(float(item["action_gap_reduction"]) < 0 for item in items))
        out.append(rec)
    return out


def make_plots(overall: list[dict[str, Any]]) -> None:
    for method in ["sensitive_basis", "delta_basis", "random_basis"]:
        rows = sorted([r for r in overall if r["subspace_method"] == method], key=lambda r: int(r["rank_k"]))
        if not rows:
            continue
        plt.plot(
            [int(r["rank_k"]) for r in rows],
            [float(r["gap_to_sim_chunk_mean_l2_mean"]) for r in rows],
            marker="o",
            label=method,
        )
    plt.xscale("log", base=2)
    plt.xlabel("rank k")
    plt.ylabel("gap to Sim chunk mean L2")
    plt.title("LOO Low-Rank Subspace Correction")
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "low_rank_subspace_gap_by_k.png", dpi=160)
    plt.close()

    for method in ["sensitive_basis", "delta_basis", "random_basis"]:
        rows = sorted([r for r in overall if r["subspace_method"] == method], key=lambda r: int(r["rank_k"]))
        if not rows:
            continue
        plt.plot(
            [int(r["rank_k"]) for r in rows],
            [float(r["repr_gap_reduction_ratio_mean"]) for r in rows],
            marker="o",
            label=method,
        )
    plt.xscale("log", base=2)
    plt.xlabel("rank k")
    plt.ylabel("representation reduction ratio")
    plt.title("LOO Low-Rank Subspace Representation Reduction")
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "low_rank_subspace_repr_by_k.png", dpi=160)
    plt.close()


def make_report(overall: list[dict[str, Any]]) -> None:
    lines = []
    for row in overall:
        lines.append(
            f"| {row['subspace_method']} | {row['rank_k']} | {row['gap_to_sim_chunk_mean_l2_mean']:.6f} | "
            f"{row['repr_gap_reduction_ratio_mean']:.6f} | {row['improved_count']} | {row['worsened_count']} |"
        )
    report = f"""# LOO Low-Rank Policy-Sensitive Subspace

[Purpose]
Evaluate whether a small train-estimated policy-sensitive subspace can approximate the Action Gap reduction of full hidden correction.

[Hypothesis]
A low-rank basis built from train local sensitive directions should reduce held-out Action Gap more efficiently than random low-rank bases and possibly more efficiently than generic delta PCA.

[Inputs]
- 225 verified Real/Sim pairs.
- `action_hidden_states.input`
- `oftplus_h5_vision`, checkpoint step 28560.

[Checked]
- train-fold sensitive-projection PCA basis
- train-fold full-delta PCA basis
- random basis control
- rank sweep: `{K_VALUES}`

[Results]
| Subspace | k | Gap to Sim chunk mean L2 | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|
{chr(10).join(lines)}

[Status]
VERIFIED offline Level-2 low-rank subspace analysis.

[Problems]
- Test correction projects held-out Real→Sim Δh, so it is not deployable as-is.
- Local sensitive vectors depend on action-head gradient and paired Sim action during analysis.
- Five episode LOO only.

[Decision]
Use this result to judge whether policy-sensitive correction can be compressed into a low-rank basis.

[Next]
Proceed to projector/vision upstream tracing.
"""
    (OUTPUT_DIR / "low_rank_sensitive_subspace_report.md").write_text(report, encoding="utf-8")


def append_log(summary: dict[str, Any]) -> None:
    best = summary["best_by_method"]
    entry = f"""

## [2026-09-16 / KST] Phase5 STEP 3 — Low-Rank Policy-Sensitive Subspace

[Purpose]
train fold에서 추정한 작은 policy-sensitive subspace만으로 held-out Action Gap을 줄일 수 있는지 확인한다.

[Hypothesis]
local sensitive direction 기반 low-rank basis는 random basis보다 효율적으로 Action Gap을 줄일 것이다.

[Inputs]
- 225 verified Real/Sim pairs.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Feature: `action_hidden_states.input`.

[Checked]
- sensitive-projection PCA basis.
- full-delta PCA basis.
- random basis control.
- rank sweep: `{K_VALUES}`.

[Changes]
- Added Phase5 STEP 3 outputs under `{OUTPUT_DIR}`.

[Commands]
`/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase5_policy_relevant_completion/scripts/low_rank_sensitive_subspace.py`

[Outputs]
- `{OUTPUT_DIR / 'low_rank_sensitive_subspace_report.md'}`
- `{OUTPUT_DIR / 'low_rank_sensitive_subspace_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'low_rank_sensitive_subspace_summary_overall.csv'}`
- `{OUTPUT_DIR / 'low_rank_sensitive_subspace_summary.json'}`

[Results]
- Best sensitive basis: k=`{best['sensitive_basis']['rank_k']}`, gap `{best['sensitive_basis']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- Best delta basis: k=`{best['delta_basis']['rank_k']}`, gap `{best['delta_basis']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- Best random basis: k=`{best['random_basis']['rank_k']}`, gap `{best['random_basis']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.

[Status]
VERIFIED offline Level-2 low-rank subspace analysis.

[Problems]
- Uses held-out Δh projection, not deployable as-is.
- Five episode LOO only.

[Decision]
Use low-rank result for deployability-oriented correction design and upstream tracing priority.

[Next]
Proceed to projector.output perturbation / layer-wise tracing.
"""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(SEED)
    real_records = records_by_frame(ANALYSIS_ROOT / "features/real/feature_manifest.json")
    sim_records = records_by_frame(ANALYSIS_ROOT / "features/sim/feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))
    phases = phase_map(PHASE_CSV)
    action_stats = read_json(CHECKPOINT / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, action_head_checkpoint = load_action_head(CHECKPOINT)

    samples = []
    for i, frame in enumerate(frames):
        real_hidden = load_npz_array(real_records[frame], FEATURE).astype(np.float64)
        sim_hidden = load_npz_array(sim_records[frame], FEATURE).astype(np.float64)
        real_action = action_chunk(real_records[frame])
        sim_action = action_chunk(sim_records[frame])
        delta = sim_hidden - real_hidden
        grad, _ = grad_direction(action_head, real_hidden.astype(np.float32), sim_action, action_stats, device)
        sensitive = project_onto(delta, -grad)
        samples.append(
            {
                "frame": frame,
                "episode_id": episode_id_from_frame(frame),
                "progress": progress_from_frame(frame),
                "planner_phase": phases.get(frame, "UNKNOWN"),
                "real_hidden": real_hidden,
                "sim_hidden": sim_hidden,
                "delta": delta,
                "sensitive": sensitive,
                "real_action": real_action,
                "sim_action": sim_action,
            }
        )
        if (i + 1) % 25 == 0:
            print(f"computed gradients {i + 1}/{len(frames)}", flush=True)

    episodes = sorted({s["episode_id"] for s in samples})
    rows = []
    shape = samples[0]["delta"].shape
    dim = int(np.prod(shape))
    for heldout in episodes:
        train = [s for s in samples if s["episode_id"] != heldout]
        test = [s for s in samples if s["episode_id"] == heldout]
        k_max = min(max(K_VALUES), len(train) - 1)
        sensitive_basis = build_basis([s["sensitive"] for s in train], k_max)
        delta_basis = build_basis([s["delta"] for s in train], k_max)
        random_bases = {k: random_basis(dim, min(k, k_max), rng) for k in K_VALUES}
        batch = []
        meta = []
        for sample in test:
            for method, basis in [("sensitive_basis", sensitive_basis), ("delta_basis", delta_basis)]:
                for k in K_VALUES:
                    corr = project_with_basis(sample["delta"], basis, shape, min(k, basis.shape[0]))
                    batch.append((sample["real_hidden"] + corr).astype(np.float32))
                    meta.append((sample, method, k, corr))
            for k in K_VALUES:
                rb = random_bases[k]
                corr = project_with_basis(sample["delta"], rb, shape, min(k, rb.shape[0]))
                batch.append((sample["real_hidden"] + corr).astype(np.float32))
                meta.append((sample, "random_basis", k, corr))
        pred_actions = predict_batch(action_head, np.concatenate(batch, axis=0), action_stats, device)
        for (sample, method, k, corr), pred_action in zip(meta, pred_actions):
            raw_gap = chunk_metrics(sample["real_action"], sample["sim_action"], "raw_gap")
            gap = chunk_metrics(pred_action, sample["sim_action"], "gap_to_sim")
            delta_norm = float(np.linalg.norm(sample["delta"].reshape(-1)))
            hidden_after_gap = float(np.linalg.norm((sample["sim_hidden"] - (sample["real_hidden"] + corr)).reshape(-1)))
            raw_chunk = float(raw_gap["raw_gap_chunk_mean_l2"])
            gap_chunk = float(gap["gap_to_sim_chunk_mean_l2"])
            rows.append(
                {
                    "frame": sample["frame"],
                    "episode_id": sample["episode_id"],
                    "heldout_episode": heldout,
                    "planner_phase": sample["planner_phase"],
                    "progress": sample["progress"],
                    "subspace_method": method,
                    "rank_k": k,
                    "delta_hidden_norm": delta_norm,
                    "shift_norm": float(np.linalg.norm(corr.reshape(-1))),
                    "hidden_after_gap": hidden_after_gap,
                    "repr_gap_reduction_ratio": (delta_norm - hidden_after_gap) / delta_norm if delta_norm > 1e-18 else 0.0,
                    **raw_gap,
                    **gap,
                    "action_gap_reduction": raw_chunk - gap_chunk,
                    "action_gap_reduction_ratio": (raw_chunk - gap_chunk) / raw_chunk if raw_chunk > 1e-18 else 0.0,
                }
            )
        print(f"processed heldout {heldout}", flush=True)

    overall = aggregate(rows, ["subspace_method", "rank_k"])
    by_phase = aggregate(rows, ["planner_phase", "subspace_method", "rank_k"])
    write_csv(OUTPUT_DIR / "low_rank_sensitive_subspace_frame_metrics.csv", rows)
    write_csv(OUTPUT_DIR / "low_rank_sensitive_subspace_summary_overall.csv", overall)
    write_csv(OUTPUT_DIR / "low_rank_sensitive_subspace_summary_by_phase.csv", by_phase)
    make_plots(overall)
    make_report(overall)
    best = {}
    for method in ["sensitive_basis", "delta_basis", "random_basis"]:
        candidates = [row for row in overall if row["subspace_method"] == method]
        best[method] = min(candidates, key=lambda r: float(r["gap_to_sim_chunk_mean_l2_mean"]))
    summary = {
        "experiment": "Phase5 STEP 3 LOO low-rank policy-sensitive subspace",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "frames": len(samples),
        "feature": FEATURE,
        "k_values": K_VALUES,
        "checkpoint": str(CHECKPOINT),
        "action_head_checkpoint": action_head_checkpoint,
        "device": str(device),
        "best_by_method": {
            method: {
                "rank_k": int(row["rank_k"]),
                "gap_to_sim_chunk_mean_l2_mean": float(row["gap_to_sim_chunk_mean_l2_mean"]),
                "repr_gap_reduction_ratio_mean": float(row["repr_gap_reduction_ratio_mean"]),
                "improved_count": int(row["improved_count"]),
                "worsened_count": int(row["worsened_count"]),
            }
            for method, row in best.items()
        },
        "outputs": {
            "frame_metrics": str(OUTPUT_DIR / "low_rank_sensitive_subspace_frame_metrics.csv"),
            "overall": str(OUTPUT_DIR / "low_rank_sensitive_subspace_summary_overall.csv"),
            "by_phase": str(OUTPUT_DIR / "low_rank_sensitive_subspace_summary_by_phase.csv"),
            "report": str(OUTPUT_DIR / "low_rank_sensitive_subspace_report.md"),
        },
        "interpretation_limits": [
            "Uses held-out delta projection, not deployable as-is.",
            "Five episode LOO only.",
            "Offline action-head evidence only.",
        ],
    }
    write_json(OUTPUT_DIR / "low_rank_sensitive_subspace_summary.json", summary)
    append_log(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:10000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
