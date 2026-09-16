#!/usr/bin/env python3
"""Token-level policy relevance at action_hidden_states.input."""

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
OUTPUT_DIR = ROOT / "lhj/phase4_policy_relevance/action_hidden_token_sensitivity"
LOG_PATH = ROOT / "lhj/작업기록.md"
OFT_REPO = ROOT / "runtime/openvla-oft"
FEATURE = "action_hidden_states.input"
DATASET_KEY = "a0509_sim_cube_pick"
ACTION_DIM = 7
CHUNK = 5
TOKEN_COUNT = 35


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


def predict_batch(action_head: torch.nn.Module, hidden_batch: np.ndarray, stats: dict[str, Any], device: torch.device) -> list[np.ndarray]:
    with torch.inference_mode():
        tensor = torch.as_tensor(hidden_batch, device=device, dtype=torch.float32)
        pred = action_head.predict_action(tensor).reshape(-1, CHUNK, ACTION_DIM).float().cpu().numpy()
    return [unnormalize_np(pred[i], stats) for i in range(pred.shape[0])]


def aggregate(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[k] for k in group_keys), []).append(row)
    metrics = [
        "token_delta_norm",
        "token_norm_fraction",
        "gap_to_sim_chunk_mean_l2",
        "gap_to_sim_chunk_max_l2",
        "gap_to_sim_first_translation_l2",
        "gap_to_sim_first_rotation_l2",
        "gap_to_sim_first_gripper_abs",
        "action_gap_reduction",
        "action_gap_reduction_ratio",
        "repr_gap_reduction_ratio",
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
        record["improved_count"] = int(sum(float(item["action_gap_reduction"]) > 0 for item in items))
        record["worsened_count"] = int(sum(float(item["action_gap_reduction"]) < 0 for item in items))
        out.append(record)
    return out


def make_plots(token_summary: list[dict[str, Any]]) -> None:
    tokens = [int(row["token_index"]) for row in token_summary]
    action_red = [float(row["action_gap_reduction_mean"]) for row in token_summary]
    repr_red = [float(row["repr_gap_reduction_ratio_mean"]) for row in token_summary]
    gap = [float(row["gap_to_sim_chunk_mean_l2_mean"]) for row in token_summary]
    gripper = [float(row["gap_to_sim_first_gripper_abs_mean"]) for row in token_summary]

    plt.figure(figsize=(10, 4.8))
    plt.bar(tokens, action_red)
    plt.xlabel("token index")
    plt.ylabel("mean action gap reduction")
    plt.title("Token Δh Only: Action Gap Reduction")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "token_action_gap_reduction.png", dpi=160)
    plt.close()

    plt.figure(figsize=(6.2, 5.2))
    plt.scatter(repr_red, action_red)
    for token, x, y in zip(tokens, repr_red, action_red):
        if y >= np.percentile(action_red, 85):
            plt.text(x, y, str(token), fontsize=8)
    plt.xlabel("representation gap reduction ratio")
    plt.ylabel("mean action gap reduction")
    plt.title("Token Representation Reduction vs Action Effect")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "token_repr_vs_action_effect.png", dpi=160)
    plt.close()

    plt.figure(figsize=(10, 4.8))
    plt.plot(tokens, gap, marker="o", label="chunk mean L2")
    plt.plot(tokens, gripper, marker="o", label="first gripper abs")
    plt.xlabel("token index")
    plt.ylabel("gap to Sim")
    plt.title("Token Δh Only: Remaining Action Gap")
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "token_remaining_action_gap.png", dpi=160)
    plt.close()


def make_report(token_summary: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    top = sorted(token_summary, key=lambda r: float(r["action_gap_reduction_mean"]), reverse=True)[:10]
    lines = []
    for row in top:
        lines.append(
            f"| {row['token_index']} | {float(row['action_gap_reduction_mean']):.6f} | "
            f"{float(row['gap_to_sim_chunk_mean_l2_mean']):.6f} | "
            f"{float(row['repr_gap_reduction_ratio_mean']):.6f} | "
            f"{float(row['token_norm_fraction_mean']):.6f} | "
            f"{row['improved_count']} | {row['worsened_count']} |"
        )
    report = f"""# Action-Hidden Token-Level Sensitivity

Experiment:
Apply the Real→Sim Δh of one `action_hidden_states.input` token at a time and measure the action-head effect.

Purpose:
Identify whether policy-relevant Real/Sim hidden differences are concentrated in specific action-hidden tokens/components.

Hypothesis:
Some tokens will reduce Action Gap more than others, and token norm or representation-distance reduction alone will not fully explain policy impact.

Input:
- 225 verified Real/Sim pairs.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Layer: `action_hidden_states.input`, token count 35.

Method:
For each frame and token `t`, evaluate `h_real` with only token `t` replaced by `h_real[t] + (h_sim[t] - h_real[t])`. All other tokens remain Real.

Control:
The control is the full no-correction raw action gap and comparison across all same-layer tokens. This does not assume token index maps directly to a physical image region.

Metrics:
Action gap reduction, remaining chunk mean Action Gap, first-action translation/rotation/gripper gaps, representation gap reduction, token norm fraction.

Result:
Top 10 tokens by mean Action Gap reduction:
| Token | Mean action gap reduction | Remaining chunk mean gap | Repr reduction ratio | Token norm fraction | Improved | Worsened |
|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(lines)}

Interpretation:
This is token-level policy-relevance evidence at the action-facing hidden layer. A token with high action effect but modest representation reduction is more policy-relevant than a token that only explains large latent distance.

Status:
VERIFIED offline Level-2 token/component analysis for the existing `oftplus_h5_vision` 225-pair dataset.

Limitation:
Token index semantics are not mapped to physical image regions. This is action-hidden token relevance, not direct visual patch attribution.

Next decision:
Use top tokens/components for targeted correction ablation and compare against whole-hidden progress-conditioned correction.
"""
    (OUTPUT_DIR / "action_hidden_token_sensitivity_report.md").write_text(report, encoding="utf-8")


def append_log(summary: dict[str, Any]) -> None:
    top = summary["top_tokens_by_action_reduction"][:5]
    top_line = ", ".join(f"{item['token_index']}:{item['action_gap_reduction_mean']:.6f}" for item in top)
    entry = f"""

## [2026-09-16 / KST] Action-Hidden Token Sensitivity

[Purpose]
`action_hidden_states.input`의 35개 token 중 어떤 token의 Real→Sim difference가 Action Gap에 가장 큰 영향을 주는지 확인한다.

[Hypothesis]
일부 token은 representation distance contribution보다 큰 action effect를 가지며, 이는 policy-relevant component 후보가 된다.

[Inputs]
- 225 verified Real/Sim pairs, 5 episodes.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Feature: `action_hidden_states.input`.
- Action head: `{summary['action_head_checkpoint']}`.

[Checked]
- Token-wise Δh-only correction for all 35 tokens.
- Overall token summary.
- Phase and episode token summary.

[Changes]
- Added token sensitivity outputs under `{OUTPUT_DIR}`.

[Commands]
`/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/action_hidden_token_sensitivity.py`

[Outputs]
- `{OUTPUT_DIR / 'action_hidden_token_sensitivity_report.md'}`
- `{OUTPUT_DIR / 'token_sensitivity_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'token_sensitivity_summary_by_token.csv'}`
- `{OUTPUT_DIR / 'token_sensitivity_summary_by_phase_token.csv'}`
- `{OUTPUT_DIR / 'token_sensitivity_summary.json'}`
- `{OUTPUT_DIR / 'token_action_gap_reduction.png'}`
- `{OUTPUT_DIR / 'token_repr_vs_action_effect.png'}`
- `{OUTPUT_DIR / 'token_remaining_action_gap.png'}`

[Results]
- Top token/action-reduction candidates: `{top_line}`.
- Full top-token table is in the report.

[Status]
VERIFIED offline Level-2 token/component evidence.

[Problems]
- Token index is not interpreted as a physical image region.
- This is action-hidden relevance, not upstream vision patch attribution.

[Decision]
Use high-action-effect tokens as candidates for targeted correction ablation.

[Next]
Run targeted top-k token correction and compare Action Gap vs representation reduction.
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
    rows: list[dict[str, Any]] = []

    for idx, frame in enumerate(frames):
        real_hidden = load_npz_array(real_records[frame], FEATURE).astype(np.float64)
        sim_hidden = load_npz_array(sim_records[frame], FEATURE).astype(np.float64)
        real_action = action_chunk(real_records[frame])
        sim_action = action_chunk(sim_records[frame])
        delta = sim_hidden - real_hidden
        total_norm = float(np.linalg.norm(delta.reshape(-1)))
        raw_gap = chunk_metrics(real_action, sim_action, "raw_gap")

        batch = []
        token_meta = []
        for token_index in range(TOKEN_COUNT):
            corr = np.zeros_like(delta)
            corr[:, token_index, :] = delta[:, token_index, :]
            token_norm = float(np.linalg.norm(corr.reshape(-1)))
            hidden = (real_hidden + corr).astype(np.float32)
            batch.append(hidden)
            token_meta.append((token_index, token_norm, corr))
        pred_actions = predict_batch(action_head, np.concatenate(batch, axis=0), action_stats, device)

        for (token_index, token_norm, corr), pred_action in zip(token_meta, pred_actions):
            gap = chunk_metrics(pred_action, sim_action, "gap_to_sim")
            hidden_after_gap = float(np.linalg.norm((sim_hidden - (real_hidden + corr)).reshape(-1)))
            raw_chunk = float(raw_gap["raw_gap_chunk_mean_l2"])
            gap_chunk = float(gap["gap_to_sim_chunk_mean_l2"])
            rows.append(
                {
                    "frame": frame,
                    "episode_id": episode_id_from_frame(frame),
                    "progress": progress_from_frame(frame),
                    "planner_phase": phases.get(frame, "UNKNOWN"),
                    "token_index": token_index,
                    "delta_hidden_norm": total_norm,
                    "token_delta_norm": token_norm,
                    "token_norm_fraction": token_norm / total_norm if total_norm > 1e-18 else 0.0,
                    "hidden_after_gap": hidden_after_gap,
                    "repr_gap_reduction_ratio": (total_norm - hidden_after_gap) / total_norm if total_norm > 1e-18 else 0.0,
                    **raw_gap,
                    **gap,
                    "action_gap_reduction": raw_chunk - gap_chunk,
                    "action_gap_reduction_ratio": (raw_chunk - gap_chunk) / raw_chunk if raw_chunk > 1e-18 else 0.0,
                }
            )
        if (idx + 1) % 25 == 0:
            print(f"processed {idx + 1}/{len(frames)} frames", flush=True)

    by_token = aggregate(rows, ["token_index"])
    by_phase_token = aggregate(rows, ["planner_phase", "token_index"])
    by_episode_token = aggregate(rows, ["episode_id", "token_index"])
    write_csv(OUTPUT_DIR / "token_sensitivity_frame_metrics.csv", rows)
    write_csv(OUTPUT_DIR / "token_sensitivity_summary_by_token.csv", by_token)
    write_csv(OUTPUT_DIR / "token_sensitivity_summary_by_phase_token.csv", by_phase_token)
    write_csv(OUTPUT_DIR / "token_sensitivity_summary_by_episode_token.csv", by_episode_token)
    make_plots(by_token)
    top = sorted(by_token, key=lambda r: float(r["action_gap_reduction_mean"]), reverse=True)[:10]
    summary = {
        "experiment": "action_hidden_states.input token-level sensitivity",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "frames": len(frames),
        "feature": FEATURE,
        "tokens": TOKEN_COUNT,
        "checkpoint": str(CHECKPOINT),
        "action_head_checkpoint": action_head_checkpoint,
        "device": str(device),
        "top_tokens_by_action_reduction": [
            {
                "token_index": int(row["token_index"]),
                "action_gap_reduction_mean": float(row["action_gap_reduction_mean"]),
                "gap_to_sim_chunk_mean_l2_mean": float(row["gap_to_sim_chunk_mean_l2_mean"]),
                "repr_gap_reduction_ratio_mean": float(row["repr_gap_reduction_ratio_mean"]),
                "token_norm_fraction_mean": float(row["token_norm_fraction_mean"]),
            }
            for row in top
        ],
        "outputs": {
            "frame_metrics": str(OUTPUT_DIR / "token_sensitivity_frame_metrics.csv"),
            "by_token": str(OUTPUT_DIR / "token_sensitivity_summary_by_token.csv"),
            "by_phase_token": str(OUTPUT_DIR / "token_sensitivity_summary_by_phase_token.csv"),
            "report": str(OUTPUT_DIR / "action_hidden_token_sensitivity_report.md"),
        },
        "interpretation_limits": [
            "Token index is not mapped to physical image regions.",
            "This is action-hidden token relevance, not direct visual patch attribution.",
            "This is offline Level-2 action evidence only.",
        ],
    }
    write_json(OUTPUT_DIR / "token_sensitivity_summary.json", summary)
    make_report(by_token, summary)
    append_log(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:8000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
