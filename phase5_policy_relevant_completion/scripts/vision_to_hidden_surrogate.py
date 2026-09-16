#!/usr/bin/env python3
"""LOO surrogate tracing from vision_backbone.output delta to action-hidden correction."""

from __future__ import annotations

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
PROJECTOR_FEATURE = "vision_backbone.output"
HIDDEN_FEATURE = "action_hidden_states.input"
RIDGES = [1e-3, 1e-2, 1e-1, 1.0, 10.0]


def flatten_mean_tokens(arr: np.ndarray) -> np.ndarray:
    # Keep this simple and robust: pooled vision delta, not token-level causal injection.
    return arr.reshape(arr.shape[0], arr.shape[1], arr.shape[2]).mean(axis=1).reshape(-1).astype(np.float64)


def fit_dual_ridge(x: np.ndarray, y: np.ndarray, ridge: float) -> np.ndarray:
    # Dual ridge: alpha = (X X^T + ridge I)^-1 Y.
    # Avoids materializing a 4096 x 143k primal matrix.
    kernel = x @ x.T
    kernel.flat[:: kernel.shape[0] + 1] += ridge
    return np.linalg.solve(kernel, y)


def aggregate(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[k] for k in group_keys), []).append(row)
    metrics = [
        "gap_to_sim_chunk_mean_l2",
        "gap_to_sim_first_translation_l2",
        "gap_to_sim_first_rotation_l2",
        "gap_to_sim_first_gripper_abs",
        "hidden_cosine_to_sensitive",
        "hidden_pred_norm",
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


def cosine(a: np.ndarray, b: np.ndarray) -> float:
    af = a.reshape(-1)
    bf = b.reshape(-1)
    denom = float(np.linalg.norm(af) * np.linalg.norm(bf))
    if denom <= 1e-12:
        return 0.0
    return float(np.dot(af, bf) / denom)


def make_plots(overall: list[dict[str, Any]]) -> None:
    rows = sorted(overall, key=lambda r: float(r["ridge"]))
    ridges = [float(r["ridge"]) for r in rows]
    gaps = [float(r["gap_to_sim_chunk_mean_l2_mean"]) for r in rows]
    cos_vals = [float(r["hidden_cosine_to_sensitive_mean"]) for r in rows]
    plt.figure(figsize=(7.2, 4.6))
    plt.plot(ridges, gaps, marker="o")
    plt.xscale("log")
    plt.xlabel("ridge")
    plt.ylabel("gap to Sim chunk mean L2")
    plt.title("Vision→Hidden Surrogate Action Gap")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "vision_to_hidden_surrogate_gap.png", dpi=160)
    plt.close()

    plt.figure(figsize=(7.2, 4.6))
    plt.plot(ridges, cos_vals, marker="o")
    plt.xscale("log")
    plt.xlabel("ridge")
    plt.ylabel("cosine(pred hidden, sensitive hidden)")
    plt.title("Vision→Hidden Surrogate Alignment")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "vision_to_hidden_surrogate_cosine.png", dpi=160)
    plt.close()


def make_report(overall: list[dict[str, Any]]) -> None:
    lines = []
    for row in sorted(overall, key=lambda r: float(r["ridge"])):
        lines.append(
            f"| {row['ridge']} | {row['gap_to_sim_chunk_mean_l2_mean']:.6f} | "
            f"{row['hidden_cosine_to_sensitive_mean']:.6f} | {row['improved_count']} | {row['worsened_count']} |"
        )
    report = f"""# Vision→Hidden Surrogate Tracing

[Purpose]
Trace whether upstream `vision_backbone.output` Real/Sim differences can predict policy-sensitive corrections at `action_hidden_states.input`.

[Hypothesis]
If policy-relevant hidden differences are already encoded in vision deltas, a train-fold linear map from pooled vision delta to sensitive hidden delta should reduce held-out Action Gap.

[Inputs]
- 225 verified Real/Sim pairs.
- `vision_backbone.output`
- `action_hidden_states.input`
- `oftplus_h5_vision`, checkpoint step 28560.

[Checked]
- LOO ridge map: pooled vision delta -> sensitive hidden delta.
- Ridge values: `{RIDGES}`.
- Evaluation by action-head output after applying predicted hidden correction.

[Results]
| Ridge | Gap to Sim chunk mean L2 | Cosine to sensitive hidden | Improved | Worsened |
|---:|---:|---:|---:|---:|
{chr(10).join(lines)}

[Status]
VERIFIED offline surrogate tracing.

[Problems]
- This is not exact full-forward projector injection.
- Projector delta is pooled over 256 tokens, so token-level upstream attribution is not resolved.
- Uses train paired data to learn mapping and held-out delta for projection/evaluation.

[Decision]
Use as evidence for whether projector differences carry action-sensitive hidden structure; do not claim causal full-forward injection.

[Next]
If surrogate is positive, expand to token/component mapping; otherwise prioritize hidden-level correction and deployable gating.
"""
    (OUTPUT_DIR / "vision_to_hidden_surrogate_report.md").write_text(report, encoding="utf-8")


def append_log(summary: dict[str, Any]) -> None:
    best = summary["best"]
    entry = f"""

## [2026-09-16 / KST] Phase5 STEP 4 — Vision→Hidden Surrogate Tracing

[Purpose]
`vision_backbone.output` Real/Sim delta가 `action_hidden_states.input`의 policy-sensitive hidden correction을 예측할 수 있는지 확인한다.

[Hypothesis]
Upstream vision delta에 policy-relevant 정보가 있다면, train fold에서 학습한 linear ridge map이 held-out Action Gap을 줄일 수 있다.

[Inputs]
- 225 verified Real/Sim pairs.
- `vision_backbone.output`, `action_hidden_states.input`.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.

[Checked]
- LOO ridge map from pooled vision delta to sensitive hidden delta.
- Ridge sweep: `{RIDGES}`.
- Action-head evaluation after predicted hidden correction.

[Changes]
- Added surrogate tracing outputs under `{OUTPUT_DIR}`.

[Commands]
`/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase5_policy_relevant_completion/scripts/vision_to_hidden_surrogate.py`

[Outputs]
- `{OUTPUT_DIR / 'vision_to_hidden_surrogate_report.md'}`
- `{OUTPUT_DIR / 'vision_to_hidden_surrogate_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'vision_to_hidden_surrogate_summary_overall.csv'}`
- `{OUTPUT_DIR / 'vision_to_hidden_surrogate_summary.json'}`

[Results]
- Best ridge: `{best['ridge']}`.
- Best gap: `{best['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- Best cosine to sensitive hidden: `{best['hidden_cosine_to_sensitive_mean']:.6f}`.

[Status]
VERIFIED offline surrogate tracing. Exact projector full-forward perturbation remains UNVERIFIED / RE-RUN REQUIRED.

[Problems]
- Not exact full VLM projector injection.
- Uses pooled vision delta.

[Decision]
Use only as upstream tensor-level tracing evidence.

[Next]
Proceed to layer-wise synthesis and decide whether further upstream token mapping is justified.
"""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    real_records = records_by_frame(ANALYSIS_ROOT / "features/real/feature_manifest.json")
    sim_records = records_by_frame(ANALYSIS_ROOT / "features/sim/feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))
    phases = phase_map(PHASE_CSV)
    action_stats = read_json(CHECKPOINT / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, action_head_checkpoint = load_action_head(CHECKPOINT)

    samples = []
    for i, frame in enumerate(frames):
        real_hidden = load_npz_array(real_records[frame], HIDDEN_FEATURE).astype(np.float64)
        sim_hidden = load_npz_array(sim_records[frame], HIDDEN_FEATURE).astype(np.float64)
        real_proj = load_npz_array(real_records[frame], PROJECTOR_FEATURE).astype(np.float64)
        sim_proj = load_npz_array(sim_records[frame], PROJECTOR_FEATURE).astype(np.float64)
        real_action = action_chunk(real_records[frame])
        sim_action = action_chunk(sim_records[frame])
        delta_hidden = sim_hidden - real_hidden
        delta_proj = sim_proj - real_proj
        grad, _ = grad_direction(action_head, real_hidden.astype(np.float32), sim_action, action_stats, device)
        sensitive = project_onto(delta_hidden, -grad)
        samples.append(
            {
                "frame": frame,
                "episode_id": episode_id_from_frame(frame),
                "progress": progress_from_frame(frame),
                "planner_phase": phases.get(frame, "UNKNOWN"),
                "projector_delta_pooled": flatten_mean_tokens(delta_proj),
                "real_hidden": real_hidden,
                "sensitive": sensitive,
                "real_action": real_action,
                "sim_action": sim_action,
            }
        )
        if (i + 1) % 25 == 0:
            print(f"prepared {i + 1}/{len(frames)}", flush=True)

    episodes = sorted({s["episode_id"] for s in samples})
    rows = []
    for heldout in episodes:
        train = [s for s in samples if s["episode_id"] != heldout]
        test = [s for s in samples if s["episode_id"] == heldout]
        x_train = np.stack([s["projector_delta_pooled"] for s in train], axis=0)
        x_mean = x_train.mean(axis=0, keepdims=True)
        x_std = x_train.std(axis=0, keepdims=True) + 1e-6
        x_train_n = (x_train - x_mean) / x_std
        y_train = np.stack([s["sensitive"].reshape(-1) for s in train], axis=0)
        y_mean = y_train.mean(axis=0, keepdims=True)
        y_train_c = y_train - y_mean
        x_test = np.stack([s["projector_delta_pooled"] for s in test], axis=0)
        x_test_n = (x_test - x_mean) / x_std
        k_test = x_test_n @ x_train_n.T
        batch = []
        meta = []
        for ridge in RIDGES:
            alpha = fit_dual_ridge(x_train_n, y_train_c, ridge)
            pred_all = k_test @ alpha + y_mean
            for sample, pred_flat in zip(test, pred_all):
                pred = pred_flat.reshape(sample["sensitive"].shape)
                batch.append((sample["real_hidden"] + pred).astype(np.float32))
                meta.append((sample, ridge, pred))
        pred_actions = predict_batch(action_head, np.concatenate(batch, axis=0), action_stats, device)
        for (sample, ridge, pred_hidden), pred_action in zip(meta, pred_actions):
            raw_gap = chunk_metrics(sample["real_action"], sample["sim_action"], "raw_gap")
            gap = chunk_metrics(pred_action, sample["sim_action"], "gap_to_sim")
            raw_chunk = float(raw_gap["raw_gap_chunk_mean_l2"])
            gap_chunk = float(gap["gap_to_sim_chunk_mean_l2"])
            rows.append(
                {
                    "frame": sample["frame"],
                    "episode_id": sample["episode_id"],
                    "heldout_episode": heldout,
                    "planner_phase": sample["planner_phase"],
                    "progress": sample["progress"],
                    "ridge": ridge,
                    "hidden_cosine_to_sensitive": cosine(pred_hidden, sample["sensitive"]),
                    "hidden_pred_norm": float(np.linalg.norm(pred_hidden.reshape(-1))),
                    **raw_gap,
                    **gap,
                    "action_gap_reduction": raw_chunk - gap_chunk,
                    "action_gap_reduction_ratio": (raw_chunk - gap_chunk) / raw_chunk if raw_chunk > 1e-18 else 0.0,
                }
            )
        print(f"processed heldout {heldout}", flush=True)

    overall = aggregate(rows, ["ridge"])
    by_phase = aggregate(rows, ["planner_phase", "ridge"])
    write_csv(OUTPUT_DIR / "vision_to_hidden_surrogate_frame_metrics.csv", rows)
    write_csv(OUTPUT_DIR / "vision_to_hidden_surrogate_summary_overall.csv", overall)
    write_csv(OUTPUT_DIR / "vision_to_hidden_surrogate_summary_by_phase.csv", by_phase)
    make_plots(overall)
    make_report(overall)
    best = min(overall, key=lambda r: float(r["gap_to_sim_chunk_mean_l2_mean"]))
    summary = {
        "experiment": "Phase5 STEP 4 vision-to-hidden surrogate tracing",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "frames": len(samples),
        "vision_feature": PROJECTOR_FEATURE,
        "hidden_feature": HIDDEN_FEATURE,
        "ridges": RIDGES,
        "checkpoint": str(CHECKPOINT),
        "action_head_checkpoint": action_head_checkpoint,
        "device": str(device),
        "best": {
            "ridge": float(best["ridge"]),
            "gap_to_sim_chunk_mean_l2_mean": float(best["gap_to_sim_chunk_mean_l2_mean"]),
            "hidden_cosine_to_sensitive_mean": float(best["hidden_cosine_to_sensitive_mean"]),
            "improved_count": int(best["improved_count"]),
            "worsened_count": int(best["worsened_count"]),
        },
        "outputs": {
            "frame_metrics": str(OUTPUT_DIR / "vision_to_hidden_surrogate_frame_metrics.csv"),
            "overall": str(OUTPUT_DIR / "vision_to_hidden_surrogate_summary_overall.csv"),
            "by_phase": str(OUTPUT_DIR / "vision_to_hidden_surrogate_summary_by_phase.csv"),
            "report": str(OUTPUT_DIR / "vision_to_hidden_surrogate_report.md"),
        },
        "interpretation_limits": [
            "Surrogate tensor-level tracing, not exact full-forward projector injection.",
            "Pooled vision deltas only.",
            "Offline action-head evidence only.",
        ],
    }
    write_json(OUTPUT_DIR / "vision_to_hidden_surrogate_summary.json", summary)
    append_log(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:10000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
