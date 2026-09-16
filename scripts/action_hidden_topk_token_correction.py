#!/usr/bin/env python3
"""Top-k token targeted correction at action_hidden_states.input."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from action_hidden_token_sensitivity import (  # noqa: E402
    ACTION_DIM,
    ANALYSIS_ROOT,
    CHECKPOINT,
    CHUNK,
    DATASET_KEY,
    FEATURE,
    LOG_PATH,
    OUTPUT_DIR as TOKEN_OUTPUT_DIR,
    PHASE_CSV,
    ROOT,
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


OUTPUT_DIR = ROOT / "lhj/phase4_policy_relevance/action_hidden_topk_token_correction"
TOKEN_SUMMARY = TOKEN_OUTPUT_DIR / "token_sensitivity_summary_by_token.csv"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def ranked_tokens() -> list[int]:
    rows = read_csv(TOKEN_SUMMARY)
    rows = sorted(rows, key=lambda r: float(r["action_gap_reduction_mean"]), reverse=True)
    return [int(row["token_index"]) for row in rows]


def method_sets(rank: list[int]) -> dict[str, list[int]]:
    bottom = list(reversed(rank))
    return {
        "no_correction": [],
        "top1": rank[:1],
        "top2": rank[:2],
        "top3": rank[:3],
        "top5": rank[:5],
        "top10": rank[:10],
        "top15": rank[:15],
        "top20": rank[:20],
        "bottom5": bottom[:5],
        "bottom10": bottom[:10],
        "all35_full_delta": rank,
    }


def aggregate_topk(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[k] for k in group_keys), []).append(row)
    metrics = [
        "token_count",
        "hidden_shift_norm",
        "repr_gap_reduction_ratio",
        "gap_to_sim_chunk_mean_l2",
        "gap_to_sim_chunk_max_l2",
        "gap_to_sim_first_translation_l2",
        "gap_to_sim_first_rotation_l2",
        "gap_to_sim_first_gripper_abs",
        "action_gap_reduction",
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
        record["improved_count"] = int(sum(float(item["action_gap_reduction"]) > 0 for item in items))
        record["worsened_count"] = int(sum(float(item["action_gap_reduction"]) < 0 for item in items))
        out.append(record)
    return out


def summarize_by_method(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return aggregate_topk(rows, ["method"])


def plot_results(overall: list[dict[str, Any]]) -> None:
    order = ["no_correction", "top1", "top2", "top3", "top5", "top10", "top15", "top20", "all35_full_delta", "bottom5", "bottom10"]
    rows = {row["method"]: row for row in overall}
    methods = [m for m in order if m in rows]
    gaps = [float(rows[m]["gap_to_sim_chunk_mean_l2_mean"]) for m in methods]
    repr_red = [float(rows[m]["repr_gap_reduction_ratio_mean"]) for m in methods]
    raw = gaps[0]
    action_red = [(raw - gap) / raw if raw > 1e-18 else 0.0 for gap in gaps]

    plt.figure(figsize=(10.5, 4.8))
    x = np.arange(len(methods))
    plt.bar(x - 0.18, action_red, width=0.36, label="action reduction")
    plt.bar(x + 0.18, repr_red, width=0.36, label="representation reduction")
    plt.xticks(x, methods, rotation=25, ha="right")
    plt.ylabel("reduction ratio")
    plt.title("Top-k Token Correction: Action vs Representation")
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "topk_action_vs_representation_reduction.png", dpi=160)
    plt.close()

    plt.figure(figsize=(10.5, 4.8))
    plt.plot(methods, gaps, marker="o")
    plt.xticks(rotation=25, ha="right")
    plt.ylabel("gap to Sim chunk mean L2")
    plt.title("Top-k Token Correction: Remaining Action Gap")
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "topk_remaining_action_gap.png", dpi=160)
    plt.close()


def make_report(overall: list[dict[str, Any]], rank: list[int]) -> None:
    rows = {row["method"]: row for row in overall}
    order = ["no_correction", "top1", "top2", "top3", "top5", "top10", "top15", "top20", "bottom5", "bottom10", "all35_full_delta"]
    raw = float(rows["no_correction"]["gap_to_sim_chunk_mean_l2_mean"])
    lines = []
    for method in order:
        row = rows[method]
        gap = float(row["gap_to_sim_chunk_mean_l2_mean"])
        action_red = (raw - gap) / raw if raw > 1e-18 else 0.0
        lines.append(
            f"| {method} | {float(row['token_count_mean']):.0f} | {gap:.6f} | "
            f"{action_red:.6f} | {float(row['repr_gap_reduction_ratio_mean']):.6f} | "
            f"{row['improved_count']} | {row['worsened_count']} |"
        )
    report = f"""# Top-k Action-Hidden Token Correction

Experiment:
Apply Real→Sim Δh only to the most action-effective `action_hidden_states.input` tokens.

Purpose:
Test whether targeted token correction can reduce Action Gap without requiring whole-representation alignment.

Hypothesis:
Top-k action-sensitive token correction should reduce Action Gap more efficiently than bottom-k token correction and should expose a tradeoff between representation reduction and action reduction.

Input:
- 225 verified Real/Sim pairs.
- Token ranking from `{TOKEN_SUMMARY}`.
- Ranked tokens: `{rank}`.

Method:
For each frame, apply Δh only to selected token sets: top1/top2/top3/top5/top10/top15/top20, bottom5/bottom10, and all35 full Δh.

Metrics:
Remaining Action Gap, aggregate action reduction, representation reduction, improved/worsened frame count.

Result:
| Method | Tokens | Gap to Sim chunk mean L2 | Aggregate action reduction | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(lines)}

Interpretation:
Top-k token correction tests deployability-relevant targeting more directly than single-token perturbation. If top-k improves Action Gap more efficiently than bottom-k at comparable or lower representation reduction, this supports selective policy-relevant correction.

Status:
VERIFIED offline Level-2 targeted-token correction.

Limitation:
Ranking is estimated and evaluated on the same 225-pair dataset, so this is not held-out generalization. Token index still has no physical-region interpretation.

Next decision:
Run held-out or fold-wise token ranking/correction if this targeted effect is strong enough; otherwise prioritize gradient/progress-conditioned correction.
"""
    (OUTPUT_DIR / "topk_token_correction_report.md").write_text(report, encoding="utf-8")


def append_log(summary: dict[str, Any]) -> None:
    key = summary["overall_key_results"]
    entry = f"""

## [2026-09-16 / KST] Top-k Action-Hidden Token Correction

[Purpose]
Action effect가 큰 token만 선택적으로 Real→Sim correction 했을 때 전체 representation alignment 없이 Action Gap을 줄일 수 있는지 확인한다.

[Hypothesis]
Top-k token correction은 bottom-k token correction보다 Action Gap을 더 효율적으로 줄일 것이다.

[Inputs]
- 225 verified Real/Sim pairs.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- Feature: `action_hidden_states.input`.
- Token ranking source: `{TOKEN_SUMMARY}`.

[Checked]
- Top1/top2/top3/top5/top10/top15/top20 token correction.
- Bottom5/bottom10 control.
- All35 full Δh sanity check.

[Changes]
- Added top-k token correction outputs under `{OUTPUT_DIR}`.

[Commands]
`/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/action_hidden_topk_token_correction.py`

[Outputs]
- `{OUTPUT_DIR / 'topk_token_correction_report.md'}`
- `{OUTPUT_DIR / 'topk_token_correction_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'topk_token_correction_summary_overall.csv'}`
- `{OUTPUT_DIR / 'topk_token_correction_summary.json'}`
- `{OUTPUT_DIR / 'topk_action_vs_representation_reduction.png'}`
- `{OUTPUT_DIR / 'topk_remaining_action_gap.png'}`

[Results]
- no correction gap: `{key['no_correction']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- top5 gap: `{key['top5']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- top10 gap: `{key['top10']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- bottom10 gap: `{key['bottom10']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- all35 full Δh gap: `{key['all35_full_delta']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.

[Status]
VERIFIED offline Level-2 targeted-token correction.

[Problems]
- Ranking and evaluation use the same dataset; held-out token-ranking validation remains unresolved.
- Token index is not physical-region attribution.

[Decision]
Use this as within-dataset targeted correction evidence only.

[Next]
If effect is strong, run leave-one-episode-out token ranking/correction; otherwise continue with gradient/progress-conditioned correction.
"""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    rank = ranked_tokens()
    sets = method_sets(rank)
    real_records = records_by_frame(ANALYSIS_ROOT / "features/real/feature_manifest.json")
    sim_records = records_by_frame(ANALYSIS_ROOT / "features/sim/feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))
    phases = phase_map(PHASE_CSV)
    action_stats = read_json(CHECKPOINT / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, action_head_checkpoint = load_action_head(CHECKPOINT)
    rows: list[dict[str, Any]] = []

    method_names = list(sets)
    for idx, frame in enumerate(frames):
        real_hidden = load_npz_array(real_records[frame], FEATURE).astype(np.float64)
        sim_hidden = load_npz_array(sim_records[frame], FEATURE).astype(np.float64)
        real_action = action_chunk(real_records[frame])
        sim_action = action_chunk(sim_records[frame])
        delta = sim_hidden - real_hidden
        total_norm = float(np.linalg.norm(delta.reshape(-1)))
        raw_gap = chunk_metrics(real_action, sim_action, "raw_gap")
        batch = []
        corr_meta = []
        for method in method_names:
            tokens = sets[method]
            corr = np.zeros_like(delta)
            if tokens:
                corr[:, tokens, :] = delta[:, tokens, :]
            batch.append((real_hidden + corr).astype(np.float32))
            corr_meta.append((method, tokens, corr))
        pred_actions = predict_batch(action_head, np.concatenate(batch, axis=0), action_stats, device)
        for (method, tokens, corr), pred_action in zip(corr_meta, pred_actions):
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
                    "method": method,
                    "token_count": len(tokens),
                    "tokens": ",".join(str(t) for t in tokens),
                    "delta_hidden_norm": total_norm,
                    "hidden_shift_norm": float(np.linalg.norm(corr.reshape(-1))),
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

    overall = summarize_by_method(rows)
    by_phase = aggregate_topk(rows, ["planner_phase", "method"])
    write_csv(OUTPUT_DIR / "topk_token_correction_frame_metrics.csv", rows)
    write_csv(OUTPUT_DIR / "topk_token_correction_summary_overall.csv", overall)
    write_csv(OUTPUT_DIR / "topk_token_correction_summary_by_phase.csv", by_phase)
    plot_results(overall)
    make_report(overall, rank)
    key = {
        row["method"]: {
            "gap_to_sim_chunk_mean_l2_mean": float(row["gap_to_sim_chunk_mean_l2_mean"]),
            "repr_gap_reduction_ratio_mean": float(row["repr_gap_reduction_ratio_mean"]),
            "improved_count": int(row["improved_count"]),
            "worsened_count": int(row["worsened_count"]),
        }
        for row in overall
    }
    summary = {
        "experiment": "top-k action_hidden_states.input token correction",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "frames": len(frames),
        "feature": FEATURE,
        "ranked_tokens": rank,
        "methods": sets,
        "checkpoint": str(CHECKPOINT),
        "action_head_checkpoint": action_head_checkpoint,
        "device": str(device),
        "overall_key_results": key,
        "outputs": {
            "frame_metrics": str(OUTPUT_DIR / "topk_token_correction_frame_metrics.csv"),
            "overall": str(OUTPUT_DIR / "topk_token_correction_summary_overall.csv"),
            "by_phase": str(OUTPUT_DIR / "topk_token_correction_summary_by_phase.csv"),
            "report": str(OUTPUT_DIR / "topk_token_correction_report.md"),
        },
        "interpretation_limits": [
            "Token ranking and evaluation use the same 225-pair dataset.",
            "Token index is not physical-region attribution.",
            "This is offline Level-2 action evidence only.",
        ],
    }
    write_json(OUTPUT_DIR / "topk_token_correction_summary.json", summary)
    append_log(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:8000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
