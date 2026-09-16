#!/usr/bin/env python3
"""Leave-one-episode-out top-k token correction."""

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

from action_hidden_topk_token_correction import aggregate_topk  # noqa: E402
from action_hidden_token_sensitivity import (  # noqa: E402
    ANALYSIS_ROOT,
    CHECKPOINT,
    DATASET_KEY,
    FEATURE,
    LOG_PATH,
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


OUTPUT_DIR = ROOT / "lhj/phase4_policy_relevance/action_hidden_loo_topk_token_correction"
TOKEN_FRAME_METRICS = ROOT / "lhj/phase4_policy_relevance/action_hidden_token_sensitivity/token_sensitivity_frame_metrics.csv"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def train_rankings() -> dict[str, list[int]]:
    rows = read_csv(TOKEN_FRAME_METRICS)
    episodes = sorted({row["episode_id"] for row in rows})
    rankings = {}
    for heldout in episodes:
        by_token: dict[int, list[float]] = {}
        for row in rows:
            if row["episode_id"] == heldout:
                continue
            by_token.setdefault(int(row["token_index"]), []).append(float(row["action_gap_reduction"]))
        ranked = sorted(by_token, key=lambda t: float(np.mean(by_token[t])), reverse=True)
        rankings[heldout] = ranked
    return rankings


def method_sets(rank: list[int]) -> dict[str, list[int]]:
    bottom = list(reversed(rank))
    return {
        "no_correction": [],
        "loo_top5": rank[:5],
        "loo_top10": rank[:10],
        "loo_top15": rank[:15],
        "loo_top20": rank[:20],
        "loo_bottom10": bottom[:10],
        "all35_full_delta": rank,
    }


def plot_results(overall: list[dict[str, Any]]) -> None:
    rows = {row["method"]: row for row in overall}
    order = ["no_correction", "loo_top5", "loo_top10", "loo_top15", "loo_top20", "loo_bottom10", "all35_full_delta"]
    methods = [m for m in order if m in rows]
    gaps = [float(rows[m]["gap_to_sim_chunk_mean_l2_mean"]) for m in methods]
    repr_red = [float(rows[m]["repr_gap_reduction_ratio_mean"]) for m in methods]
    raw = gaps[0]
    action_red = [(raw - gap) / raw if raw > 1e-18 else 0.0 for gap in gaps]
    x = np.arange(len(methods))
    plt.figure(figsize=(9.8, 4.8))
    plt.bar(x - 0.18, action_red, width=0.36, label="action reduction")
    plt.bar(x + 0.18, repr_red, width=0.36, label="representation reduction")
    plt.xticks(x, methods, rotation=25, ha="right")
    plt.ylabel("reduction ratio")
    plt.title("LOO Top-k Token Correction")
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "loo_topk_action_vs_representation_reduction.png", dpi=160)
    plt.close()


def make_report(overall: list[dict[str, Any]], by_fold: list[dict[str, Any]], rankings: dict[str, list[int]]) -> None:
    rows = {row["method"]: row for row in overall}
    order = ["no_correction", "loo_top5", "loo_top10", "loo_top15", "loo_top20", "loo_bottom10", "all35_full_delta"]
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
    fold_lines = []
    for row in by_fold:
        if row["method"] not in {"loo_top10", "loo_bottom10"}:
            continue
        fold_lines.append(
            f"| {row['heldout_episode']} | {row['method']} | {float(row['gap_to_sim_chunk_mean_l2_mean']):.6f} | "
            f"{float(row['repr_gap_reduction_ratio_mean']):.6f} |"
        )
    report = f"""# Leave-One-Episode-Out Top-k Token Correction

Experiment:
Rank action-hidden tokens on 4 training episodes, then evaluate top-k token correction on the held-out episode.

Purpose:
Check whether token-level policy relevance survives episode-level holdout rather than only fitting the same 225-pair dataset.

Hypothesis:
LOO top-k token correction should outperform LOO bottom-k correction on held-out episodes if token ranking has generalizable policy relevance.

Input:
- Token sensitivity frame metrics: `{TOKEN_FRAME_METRICS}`.
- 225 verified Real/Sim pairs.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.

Method:
For each held-out episode, rank tokens by mean action-gap reduction on the other four episodes. Apply top5/top10/top15/top20 and bottom10 correction on the held-out episode only.

Result:
| Method | Tokens | Gap to Sim chunk mean L2 | Aggregate action reduction | Repr reduction ratio | Improved | Worsened |
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(lines)}

Fold check:
| Heldout episode | Method | Gap to Sim chunk mean L2 | Repr reduction ratio |
|---|---|---:|---:|
{chr(10).join(fold_lines)}

Token rankings:
```json
{json.dumps(rankings, ensure_ascii=False, indent=2)}
```

Interpretation:
This is stricter than within-dataset top-k correction because test episode rankings do not use test episode token effects. If top-k only weakly outperforms bottom-k, token-index targeting is less robust than gradient/progress-conditioned hidden correction.

Status:
VERIFIED offline Level-2 held-out token-correction analysis.

Limitation:
Episode-level holdout has only five folds and still uses the same task family/checkpoint. Token indices are not physical-region attributions.

Next decision:
Use LOO result to decide whether token-level targeted correction is worth expanding, or whether correction should target gradient/progress-sensitive hidden directions instead.
"""
    (OUTPUT_DIR / "loo_topk_token_correction_report.md").write_text(report, encoding="utf-8")


def append_log(summary: dict[str, Any]) -> None:
    key = summary["overall_key_results"]
    entry = f"""

## [2026-09-16 / KST] LOO Top-k Token Correction

[Purpose]
Token-level ranking/correction이 같은 dataset 내부 효과인지, episode-level holdout에서도 유지되는지 확인한다.

[Hypothesis]
4개 train episode에서 고른 top-k token은 held-out episode에서 bottom-k보다 Action Gap을 더 줄여야 한다.

[Inputs]
- 225 verified Real/Sim pairs.
- Token sensitivity frame metrics: `{TOKEN_FRAME_METRICS}`.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.

[Checked]
- Leave-one-episode-out token ranking.
- Held-out top5/top10/top15/top20 correction.
- Held-out bottom10 control.
- All35 full Δh sanity check.

[Changes]
- Added LOO top-k token correction outputs under `{OUTPUT_DIR}`.

[Commands]
`/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/scripts/action_hidden_loo_topk_token_correction.py`

[Outputs]
- `{OUTPUT_DIR / 'loo_topk_token_correction_report.md'}`
- `{OUTPUT_DIR / 'loo_topk_token_correction_frame_metrics.csv'}`
- `{OUTPUT_DIR / 'loo_topk_token_correction_summary_overall.csv'}`
- `{OUTPUT_DIR / 'loo_topk_token_correction_summary_by_fold.csv'}`
- `{OUTPUT_DIR / 'loo_topk_token_correction_summary.json'}`

[Results]
- no correction gap: `{key['no_correction']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- loo_top10 gap: `{key['loo_top10']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- loo_bottom10 gap: `{key['loo_bottom10']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- loo_top20 gap: `{key['loo_top20']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.
- all35 full Δh gap: `{key['all35_full_delta']['gap_to_sim_chunk_mean_l2_mean']:.6f}`.

[Status]
VERIFIED offline Level-2 held-out token correction.

[Problems]
- Only five episode folds.
- Token ranking remains action-hidden token ranking, not physical image attribution.

[Decision]
Use this to judge whether token-index targeting is robust enough for further expansion.

[Next]
Proceed with gradient/progress-conditioned hidden correction if token targeting is weak or diffuse.
"""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    rankings = train_rankings()
    real_records = records_by_frame(ANALYSIS_ROOT / "features/real/feature_manifest.json")
    sim_records = records_by_frame(ANALYSIS_ROOT / "features/sim/feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))
    phases = phase_map(PHASE_CSV)
    action_stats = read_json(CHECKPOINT / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, action_head_checkpoint = load_action_head(CHECKPOINT)
    rows: list[dict[str, Any]] = []

    for idx, frame in enumerate(frames):
        episode = episode_id_from_frame(frame)
        sets = method_sets(rankings[episode])
        real_hidden = load_npz_array(real_records[frame], FEATURE).astype(np.float64)
        sim_hidden = load_npz_array(sim_records[frame], FEATURE).astype(np.float64)
        real_action = action_chunk(real_records[frame])
        sim_action = action_chunk(sim_records[frame])
        delta = sim_hidden - real_hidden
        total_norm = float(np.linalg.norm(delta.reshape(-1)))
        raw_gap = chunk_metrics(real_action, sim_action, "raw_gap")
        batch = []
        meta = []
        for method, tokens in sets.items():
            corr = np.zeros_like(delta)
            if tokens:
                corr[:, tokens, :] = delta[:, tokens, :]
            batch.append((real_hidden + corr).astype(np.float32))
            meta.append((method, tokens, corr))
        pred_actions = predict_batch(action_head, np.concatenate(batch, axis=0), action_stats, device)
        for (method, tokens, corr), pred_action in zip(meta, pred_actions):
            gap = chunk_metrics(pred_action, sim_action, "gap_to_sim")
            hidden_after_gap = float(np.linalg.norm((sim_hidden - (real_hidden + corr)).reshape(-1)))
            raw_chunk = float(raw_gap["raw_gap_chunk_mean_l2"])
            gap_chunk = float(gap["gap_to_sim_chunk_mean_l2"])
            rows.append(
                {
                    "frame": frame,
                    "episode_id": episode,
                    "heldout_episode": episode,
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

    overall = aggregate_topk(rows, ["method"])
    by_fold = aggregate_topk(rows, ["heldout_episode", "method"])
    write_csv(OUTPUT_DIR / "loo_topk_token_correction_frame_metrics.csv", rows)
    write_csv(OUTPUT_DIR / "loo_topk_token_correction_summary_overall.csv", overall)
    write_csv(OUTPUT_DIR / "loo_topk_token_correction_summary_by_fold.csv", by_fold)
    plot_results(overall)
    make_report(overall, by_fold, rankings)
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
        "experiment": "leave-one-episode-out top-k action_hidden token correction",
        "policy_scope": "oftplus_h5_vision offline 225-pair dataset only",
        "frames": len(frames),
        "feature": FEATURE,
        "rankings": rankings,
        "checkpoint": str(CHECKPOINT),
        "action_head_checkpoint": action_head_checkpoint,
        "device": str(device),
        "overall_key_results": key,
        "outputs": {
            "frame_metrics": str(OUTPUT_DIR / "loo_topk_token_correction_frame_metrics.csv"),
            "overall": str(OUTPUT_DIR / "loo_topk_token_correction_summary_overall.csv"),
            "by_fold": str(OUTPUT_DIR / "loo_topk_token_correction_summary_by_fold.csv"),
            "report": str(OUTPUT_DIR / "loo_topk_token_correction_report.md"),
        },
        "interpretation_limits": [
            "Only five episode-level folds.",
            "Token index is not physical-region attribution.",
            "This is offline Level-2 action evidence only.",
        ],
    }
    write_json(OUTPUT_DIR / "loo_topk_token_correction_summary.json", summary)
    append_log(summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2)[:8000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
