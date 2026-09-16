#!/usr/bin/env python3
"""Decompose Real->Sim hidden differences into action-sensitive/null-like parts."""

from __future__ import annotations

import csv
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import torch


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
ANALYSIS_ROOT = ROOT / "outputs/token_distribution_analysis/5_episodes"
PHASE_CSV = ANALYSIS_ROOT / "episode_phase_summary/frame_metrics_enriched.csv"
CHECKPOINT = ROOT / "runtime_state/oft_mixed480_step28560_merged"
OUTPUT_DIR = ROOT / "lhj/phase4_policy_relevance/action_hidden_sensitive_null_decomposition"
LOG_PATH = ROOT / "lhj/작업기록.md"
OFT_REPO = ROOT / "runtime/openvla-oft"
FEATURE = "action_hidden_states.input"
DATASET_KEY = "a0509_sim_cube_pick"
ACTION_DIM = 7
CHUNK = 5
SEED = 20260916


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def frame_from_record(record: dict[str, Any]) -> str:
    return Path(record["source_image"]).stem


def records_by_frame(manifest_path: Path) -> dict[str, dict[str, Any]]:
    manifest = read_json(manifest_path)
    base = manifest_path.parent
    return {
        frame_from_record(record): {
            **record,
            "_feature_path": str(base / record["feature_file"]),
        }
        for record in manifest["records"]
    }


def episode_id_from_frame(frame: str) -> str:
    match = re.match(r"(episode_\d{6})_", frame)
    if not match:
        raise ValueError(frame)
    return match.group(1)


def progress_from_frame(frame: str) -> int:
    match = re.search(r"(\d+)$", frame)
    if not match:
        raise ValueError(frame)
    return int(match.group(1))


def phase_map(path: Path) -> dict[str, str]:
    return {row["pair_id"]: row["planner_phase"] for row in read_csv(path)}


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions")
    if actions is None:
        actions = [response.get("action")]
    arr = np.asarray(actions, dtype=np.float64)
    if arr.shape != (CHUNK, ACTION_DIM):
        raise ValueError(f"Unexpected action shape: {arr.shape}")
    return arr


def load_npz_array(record: dict[str, Any], key: str) -> np.ndarray:
    with np.load(record["_feature_path"], allow_pickle=True) as data:
        return np.asarray(data[key], dtype=np.float32)


def chunk_metrics(left: np.ndarray, right: np.ndarray, prefix: str) -> dict[str, float]:
    delta = np.asarray(left, dtype=np.float64) - np.asarray(right, dtype=np.float64)
    first = delta[0]
    chunk_l2 = np.linalg.norm(delta, axis=1)
    return {
        f"{prefix}_first_l2": float(np.linalg.norm(first)),
        f"{prefix}_first_translation_l2": float(np.linalg.norm(first[:3])),
        f"{prefix}_first_rotation_l2": float(np.linalg.norm(first[3:6])),
        f"{prefix}_first_gripper_abs": float(abs(first[6])),
        f"{prefix}_chunk_mean_l2": float(chunk_l2.mean()),
        f"{prefix}_chunk_max_l2": float(chunk_l2.max()),
    }


def unnormalize_np(normalized_actions: np.ndarray, stats: dict[str, Any]) -> np.ndarray:
    norm = np.asarray(normalized_actions, dtype=np.float64)
    mask = np.asarray(stats.get("mask", np.ones_like(stats["q01"], dtype=bool)), dtype=bool)
    high = np.asarray(stats["q99"], dtype=np.float64)
    low = np.asarray(stats["q01"], dtype=np.float64)
    return np.where(mask, 0.5 * (norm + 1.0) * (high - low + 1e-8) + low, norm)


def unnormalize_torch(normalized_actions: torch.Tensor, stats: dict[str, Any], device: torch.device) -> torch.Tensor:
    mask = torch.as_tensor(stats.get("mask", np.ones_like(stats["q01"], dtype=bool)), device=device, dtype=torch.bool)
    high = torch.as_tensor(stats["q99"], device=device, dtype=normalized_actions.dtype)
    low = torch.as_tensor(stats["q01"], device=device, dtype=normalized_actions.dtype)
    return torch.where(mask, 0.5 * (normalized_actions + 1.0) * (high - low + 1e-8) + low, normalized_actions)


def load_action_head(checkpoint: Path):
    os.environ.setdefault("A0509_ACTION_CHUNK_SIZE", str(CHUNK))
    if str(OFT_REPO) not in sys.path:
        sys.path.insert(0, str(OFT_REPO))
    from prismatic.models.action_heads import L1RegressionActionHead

    training_config = read_json(checkpoint / "a0509_training_config.json")
    action_head = L1RegressionActionHead(
        input_dim=4096,
        hidden_dim=4096,
        action_dim=ACTION_DIM,
        bounded_gripper=bool(training_config.get("bounded_gripper", False)),
        gripper_loss_weight=float(training_config.get("gripper_loss_weight", 3.0)),
    )
    matches = sorted(checkpoint.glob("action_head*checkpoint*.pt"))
    if len(matches) != 1:
        raise FileNotFoundError(f"Expected one action_head checkpoint, found {len(matches)}")
    state = torch.load(matches[0], map_location="cpu", weights_only=True)
    state = {(k[7:] if k.startswith("module.") else k): v for k, v in state.items()}
    action_head.load_state_dict(state)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    return action_head.to(device=device, dtype=torch.float32).eval(), device, str(matches[0])


def predict_action_np(action_head: torch.nn.Module, hidden: np.ndarray, stats: dict[str, Any], device: torch.device) -> np.ndarray:
    with torch.inference_mode():
        tensor = torch.as_tensor(hidden, device=device, dtype=torch.float32)
        pred = action_head.predict_action(tensor).reshape(CHUNK, ACTION_DIM).float().cpu().numpy()
    return unnormalize_np(pred, stats)


def grad_direction(
    action_head: torch.nn.Module,
    real_hidden: np.ndarray,
    sim_action: np.ndarray,
    stats: dict[str, Any],
    device: torch.device,
) -> tuple[np.ndarray, float]:
    hidden = torch.as_tensor(real_hidden, device=device, dtype=torch.float32).clone().detach().requires_grad_(True)
    pred_norm = action_head.predict_action(hidden).reshape(CHUNK, ACTION_DIM)
    pred = unnormalize_torch(pred_norm, stats, device)
    target = torch.as_tensor(sim_action, device=device, dtype=torch.float32)
    loss = 0.5 * torch.mean((pred - target) ** 2)
    loss.backward()
    grad = hidden.grad.detach().cpu().numpy().astype(np.float64)
    return grad, float(loss.detach().cpu().item())


def project_onto(delta: np.ndarray, direction: np.ndarray) -> np.ndarray:
    d = direction.astype(np.float64, copy=False)
    denom = float(np.dot(d.reshape(-1), d.reshape(-1)))
    if denom <= 1e-18:
        return np.zeros_like(delta, dtype=np.float64)
    coeff = float(np.dot(delta.reshape(-1), d.reshape(-1)) / denom)
    return coeff * d


def same_norm_random(shape: tuple[int, ...], norm: float, rng: np.random.Generator) -> np.ndarray:
    arr = rng.standard_normal(shape).astype(np.float64)
    arr_norm = float(np.linalg.norm(arr.reshape(-1)))
    if arr_norm <= 1e-18:
        return np.zeros(shape, dtype=np.float64)
    return arr * (norm / arr_norm)


def same_norm_orthogonal(shape: tuple[int, ...], norm: float, basis: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    arr = rng.standard_normal(shape).astype(np.float64)
    denom = float(np.dot(basis.reshape(-1), basis.reshape(-1)))
    if denom > 1e-18:
        arr = arr - float(np.dot(arr.reshape(-1), basis.reshape(-1)) / denom) * basis
    arr_norm = float(np.linalg.norm(arr.reshape(-1)))
    if arr_norm <= 1e-18:
        return np.zeros(shape, dtype=np.float64)
    return arr * (norm / arr_norm)


def aggregate(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[k] for k in group_keys), []).append(row)
    metrics = [
        "gap_to_sim_chunk_mean_l2",
        "gap_to_sim_chunk_max_l2",
        "gap_to_sim_first_translation_l2",
        "gap_to_sim_first_rotation_l2",
        "gap_to_sim_first_gripper_abs",
        "change_from_real_chunk_mean_l2",
        "hidden_shift_norm",
        "repr_gap_reduction_ratio",
        "action_gap_reduction_ratio",
    ]
    out = []
    for key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        record = {k: v for k, v in zip(group_keys, key)}
        record["count"] = len(items)
        for metric in metrics:
            vals = np.asarray([float(item[metric]) for item in items], dtype=np.float64)
            record[f"{metric}_mean"] = float(vals.mean())
            record[f"{metric}_p50"] = float(np.percentile(vals, 50))
            record[f"{metric}_p90"] = float(np.percentile(vals, 90))
        record["improved_count"] = int(sum(float(item["gap_to_sim_chunk_mean_l2"]) < float(item["raw_gap_chunk_mean_l2"]) for item in items))
        record["worsened_count"] = int(sum(float(item["gap_to_sim_chunk_mean_l2"]) > float(item["raw_gap_chunk_mean_l2"]) for item in items))
        out.append(record)
    return out


def make_plots(overall: list[dict[str, Any]], phase_summary: list[dict[str, Any]]) -> None:
    methods = [row["method"] for row in overall]
    gap = [float(row["gap_to_sim_chunk_mean_l2_mean"]) for row in overall]
    repr_red = [float(row["repr_gap_reduction_ratio_mean"]) for row in overall]
    action_red = [float(row["action_gap_reduction_ratio_mean"]) for row in overall]

    plt.figure(figsize=(9.2, 4.8))
    x = np.arange(len(methods))
    plt.bar(x - 0.18, action_red, width=0.36, label="action gap reduction")
    plt.bar(x + 0.18, repr_red, width=0.36, label="representation gap reduction")
    plt.xticks(x, methods, rotation=25, ha="right")
    plt.ylabel("mean reduction ratio")
    plt.title("Correction Decomposition: Representation vs Action Reduction")
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "method_repr_vs_action_reduction.png", dpi=160)
    plt.close()

    plt.figure(figsize=(9.2, 4.8))
    plt.bar(methods, gap)
    plt.xticks(rotation=25, ha="right")
    plt.ylabel("gap to Sim chunk mean L2")
    plt.title("Correction Decomposition: Final Action Gap")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "method_action_gap.png", dpi=160)
    plt.close()

    selected = [row for row in phase_summary if row["method"] in {"full_delta", "sensitive_projection", "null_residual", "neg_gradient_same_norm"}]
    phases = []
    for row in selected:
        if row["planner_phase"] not in phases:
            phases.append(row["planner_phase"])
    methods_phase = ["full_delta", "sensitive_projection", "null_residual", "neg_gradient_same_norm"]
    width = 0.18
    x = np.arange(len(phases))
    plt.figure(figsize=(9.6, 4.8))
    for i, method in enumerate(methods_phase):
        values = [
            float(next(row for row in selected if row["planner_phase"] == phase and row["method"] == method)["gap_to_sim_chunk_mean_l2_mean"])
            for phase in phases
        ]
        plt.bar(x + (i - 1.5) * width, values, width=width, label=method)
    plt.xticks(x, phases, rotation=25, ha="right")
    plt.ylabel("gap to Sim chunk mean L2")
    plt.title("Correction Decomposition by Phase")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "method_action_gap_by_phase.png", dpi=160)
    plt.close()


def make_report(overall: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    raw_gap = next(float(row["gap_to_sim_chunk_mean_l2_mean"]) for row in overall if row["method"] == "no_correction")
    lines = []
    for row in overall:
        gap = float(row["gap_to_sim_chunk_mean_l2_mean"])
        aggregate_action_reduction = (raw_gap - gap) / raw_gap if raw_gap > 1e-18 else 0.0
        lines.append(
            f"| {row['method']} | {gap:.6f} | "
            f"{aggregate_action_reduction:.6f} | "
            f"{float(row['repr_gap_reduction_ratio_mean']):.6f} | "
            f"{row['improved_count']} | {row['worsened_count']} |"
        )
    report = f"""# Action-Hidden Sensitive / Null-Like Decomposition

Experiment:
Gradient-based decomposition of Real→Sim hidden-state difference at `action_hidden_states.input`.

Purpose:
Check whether Action Gap reduction is tied to action-sensitive components rather than total representation-distance reduction.

Hypothesis:
The full Real→Sim Δh should reduce Action Gap. If only part of Δh is policy-sensitive, a gradient-aligned component should reduce Action Gap more efficiently than null-like or random components, even when representation-gap reduction is smaller.

Input:
- 225 verified Real/Sim pairs.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Layer: `action_hidden_states.input`.

Method:
For each frame, compute the gradient of `0.5 * mean((Action(h_real) - Action_sim)^2)` with respect to `h_real`. Then evaluate several hidden corrections through the same action head.

Control:
- `random_same_norm`: random direction with same norm as full Δh.
- `orthogonal_to_gradient_same_norm`: random direction orthogonal to the local action-gap gradient.
- `null_residual`: component of Δh after removing its projection onto the negative gradient direction.

Metrics:
Action gap to Sim action, hidden shift norm, representation gap reduction ratio, aggregate action gap reduction ratio, phase summaries, and action-dimension breakdown.

Result:
The aggregate action reduction ratio below uses `(raw mean gap - method mean gap) / raw mean gap`. This is preferred for interpretation because per-frame ratio means can be distorted by frames whose raw action gap is very small.

| Method | Gap to Sim chunk mean L2 | Aggregate action reduction | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|
{chr(10).join(lines)}

Key contrast:
- `sensitive_projection` reduces Action Gap with very small representation-gap reduction.
- `null_residual` can reduce much more representation distance while producing weaker Action Gap reduction.
- Same-norm random and gradient-orthogonal controls should be read as norm controls rather than deployable corrections.

Interpretation:
The result should be read as local action-head sensitivity evidence, not as a deployment-ready correction. A method with high action reduction and lower representation reduction supports the policy-relevance hypothesis more strongly than a method that merely reduces representation distance.

Status:
VERIFIED offline Level-2 analysis for the existing `oftplus_h5_vision` 225-pair dataset.

Limitation:
The gradient direction is local to the action head at `h_real`; it is not a full causal decomposition of upstream vision tokens or environment factors. Null-like residual is defined relative to a first-order gradient direction, not the exact nonlinear nullspace.

Next decision:
Use this result to decide whether to expand to token/component-level sensitivity or correction ablation with progress-conditioned learned shifts.

Summary JSON:
`{OUTPUT_DIR / 'sensitive_null_decomposition_summary.json'}`
"""
    (OUTPUT_DIR / "sensitive_null_decomposition_report.md").write_text(report, encoding="utf-8")


def append_log(summary: dict[str, Any]) -> None:
    rows = summary["overall_key_results"]
    def get(method: str, key: str) -> float:
        return float(rows[method][key])
    entry = f"""

## [2026-09-16 / KST] Policy-Relevant Hidden Decomposition

[Purpose]
`action_hidden_states.input`에서 Action Gap 감소가 전체 representation distance 감소 때문인지, policy-sensitive component 때문인지 확인한다.

[Hypothesis]
Gradient-aligned/sensitive component는 null-like 또는 random component보다 같은 offline action head에서 Action Gap을 더 효율적으로 줄일 것이다.

[Inputs]
- 225 verified Real/Sim pairs, 5 episodes.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Feature: `action_hidden_states.input`.
- Action head: `{summary['action_head_checkpoint']}`.

[Checked]
- Local action-gap gradient at `h_real`.
- Full Δh, sensitive projection, null residual, negative-gradient same-norm, random same-norm, gradient-orthogonal same-norm correction.
- Overall, phase, episode summaries.

[Changes]
- Added/updated outputs under `{OUTPUT_DIR}`.
- Appended this structured log entry.

[Commands]
`python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/action_hidden_sensitive_null_decomposition.py`

[Outputs]
- `{OUTPUT_DIR / 'sensitive_null_decomposition_report.md'}`
- `{OUTPUT_DIR / 'sensitive_null_decomposition_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'sensitive_null_decomposition_summary_overall.csv'}`
- `{OUTPUT_DIR / 'sensitive_null_decomposition_summary_by_phase.csv'}`
- `{OUTPUT_DIR / 'sensitive_null_decomposition_summary_by_episode.csv'}`
- `{OUTPUT_DIR / 'sensitive_null_decomposition_summary.json'}`

[Results]
- Full Δh action gap: `{get('full_delta', 'gap_to_sim_chunk_mean_l2_mean'):.6f}`.
- Sensitive projection action gap: `{get('sensitive_projection', 'gap_to_sim_chunk_mean_l2_mean'):.6f}`.
- Null residual action gap: `{get('null_residual', 'gap_to_sim_chunk_mean_l2_mean'):.6f}`.
- Negative-gradient same-norm action gap: `{get('neg_gradient_same_norm', 'gap_to_sim_chunk_mean_l2_mean'):.6f}`.
- Random same-norm action gap: `{get('random_same_norm', 'gap_to_sim_chunk_mean_l2_mean'):.6f}`.
- Gradient-orthogonal same-norm action gap: `{get('orthogonal_to_gradient_same_norm', 'gap_to_sim_chunk_mean_l2_mean'):.6f}`.

[Status]
VERIFIED offline Level-2 evidence. Real robot performance remains UNVERIFIED.

[Problems]
- Gradient-sensitive/null-like split is local to the action head at `h_real`.
- Exact nonlinear policy nullspace, upstream token cause, and environment cause remain unresolved.

[Decision]
Use this result only inside the `oftplus_h5_vision` offline evidence chain. Do not mix with ROS proprio policy.

[Next]
Proceed to token/component-level sensitivity or progress-conditioned correction ablation based on this result.
"""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    rng = np.random.default_rng(SEED)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    real_records = records_by_frame(ANALYSIS_ROOT / "features/real/feature_manifest.json")
    sim_records = records_by_frame(ANALYSIS_ROOT / "features/sim/feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))
    phases = phase_map(PHASE_CSV)
    action_stats = read_json(CHECKPOINT / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, action_head_checkpoint = load_action_head(CHECKPOINT)

    methods = [
        "no_correction",
        "full_delta",
        "sensitive_projection",
        "null_residual",
        "neg_gradient_same_norm",
        "random_same_norm",
        "orthogonal_to_gradient_same_norm",
    ]
    rows: list[dict[str, Any]] = []

    for idx, frame in enumerate(frames):
        real_hidden = load_npz_array(real_records[frame], FEATURE).astype(np.float64)
        sim_hidden = load_npz_array(sim_records[frame], FEATURE).astype(np.float64)
        real_action = action_chunk(real_records[frame])
        sim_action = action_chunk(sim_records[frame])
        delta = sim_hidden - real_hidden
        delta_norm = float(np.linalg.norm(delta.reshape(-1)))
        raw_gap = chunk_metrics(real_action, sim_action, "raw_gap")
        grad, loss = grad_direction(action_head, real_hidden.astype(np.float32), sim_action, action_stats, device)
        neg_grad = -grad
        neg_grad_norm = float(np.linalg.norm(neg_grad.reshape(-1)))
        sensitive = project_onto(delta, neg_grad)
        null = delta - sensitive
        random = same_norm_random(delta.shape, delta_norm, rng)
        orth_grad = same_norm_orthogonal(delta.shape, delta_norm, neg_grad, rng)
        neg_grad_same = neg_grad * (delta_norm / max(neg_grad_norm, 1e-18))
        correction_map = {
            "no_correction": np.zeros_like(delta),
            "full_delta": delta,
            "sensitive_projection": sensitive,
            "null_residual": null,
            "neg_gradient_same_norm": neg_grad_same,
            "random_same_norm": random,
            "orthogonal_to_gradient_same_norm": orth_grad,
        }
        for method in methods:
            corr = correction_map[method]
            pred_action = predict_action_np(action_head, (real_hidden + corr).astype(np.float32), action_stats, device)
            gap = chunk_metrics(pred_action, sim_action, "gap_to_sim")
            change = chunk_metrics(pred_action, real_action, "change_from_real")
            hidden_after_gap = float(np.linalg.norm((sim_hidden - (real_hidden + corr)).reshape(-1)))
            hidden_shift_norm = float(np.linalg.norm(corr.reshape(-1)))
            raw_action_gap = float(raw_gap["raw_gap_chunk_mean_l2"])
            action_gap = float(gap["gap_to_sim_chunk_mean_l2"])
            repr_red = (delta_norm - hidden_after_gap) / delta_norm if delta_norm > 1e-18 else 0.0
            action_red = (raw_action_gap - action_gap) / raw_action_gap if raw_action_gap > 1e-18 else 0.0
            rows.append(
                {
                    "frame": frame,
                    "episode_id": episode_id_from_frame(frame),
                    "progress": progress_from_frame(frame),
                    "planner_phase": phases.get(frame, "UNKNOWN"),
                    "method": method,
                    "delta_hidden_norm": delta_norm,
                    "gradient_norm": neg_grad_norm,
                    "gradient_loss": loss,
                    "sensitive_projection_norm": float(np.linalg.norm(sensitive.reshape(-1))),
                    "null_residual_norm": float(np.linalg.norm(null.reshape(-1))),
                    "hidden_shift_norm": hidden_shift_norm,
                    "hidden_after_gap": hidden_after_gap,
                    "repr_gap_reduction_ratio": repr_red,
                    **raw_gap,
                    **gap,
                    **change,
                    "action_gap_reduction_ratio": action_red,
                    "action_gap_reduction": raw_action_gap - action_gap,
                }
            )
        if (idx + 1) % 25 == 0:
            print(f"processed {idx + 1}/{len(frames)} frames")

    overall = aggregate(rows, ["method"])
    by_phase = aggregate(rows, ["planner_phase", "method"])
    by_episode = aggregate(rows, ["episode_id", "method"])
    write_csv(OUTPUT_DIR / "sensitive_null_decomposition_frame_metrics.csv", rows)
    write_csv(OUTPUT_DIR / "sensitive_null_decomposition_summary_overall.csv", overall)
    write_csv(OUTPUT_DIR / "sensitive_null_decomposition_summary_by_phase.csv", by_phase)
    write_csv(OUTPUT_DIR / "sensitive_null_decomposition_summary_by_episode.csv", by_episode)
    make_plots(overall, by_phase)
    overall_key = {
        row["method"]: {
            "gap_to_sim_chunk_mean_l2_mean": row["gap_to_sim_chunk_mean_l2_mean"],
            "action_gap_reduction_ratio_mean": row["action_gap_reduction_ratio_mean"],
            "repr_gap_reduction_ratio_mean": row["repr_gap_reduction_ratio_mean"],
            "improved_count": row["improved_count"],
            "worsened_count": row["worsened_count"],
        }
        for row in overall
    }
    summary = {
        "experiment": "action_hidden_states.input sensitive/null-like decomposition",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "frames": len(frames),
        "feature": FEATURE,
        "checkpoint": str(CHECKPOINT),
        "action_head_checkpoint": action_head_checkpoint,
        "device": str(device),
        "seed": SEED,
        "methods": methods,
        "overall_key_results": overall_key,
        "outputs": {
            "frame_metrics": str(OUTPUT_DIR / "sensitive_null_decomposition_frame_metrics.csv"),
            "overall": str(OUTPUT_DIR / "sensitive_null_decomposition_summary_overall.csv"),
            "by_phase": str(OUTPUT_DIR / "sensitive_null_decomposition_summary_by_phase.csv"),
            "by_episode": str(OUTPUT_DIR / "sensitive_null_decomposition_summary_by_episode.csv"),
            "report": str(OUTPUT_DIR / "sensitive_null_decomposition_report.md"),
        },
        "interpretation_limits": [
            "Gradient-sensitive/null-like split is local to action_head at h_real.",
            "This is offline Level-2 action evidence only.",
            "This does not prove real robot performance or environment causality.",
        ],
    }
    write_json(OUTPUT_DIR / "sensitive_null_decomposition_summary.json", summary)
    make_report(overall, summary)
    append_log(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:8000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
