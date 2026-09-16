#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE6 = ROOT / "lhj" / "phase6_environment_attribution"
FEATURE_ROOT = PHASE6 / "01_preprocessing" / "full_forward_features"
OBS_TABLE = PHASE6 / "environment_factor_table.csv"
PAIR_MANIFEST = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes" / "paired_inputs" / "paired_manifest.json"


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions")
    if actions is None:
        actions = [response.get("action")]
    arr = np.asarray(actions, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    return arr


def pair_id_from_source(record: dict[str, Any]) -> str:
    return Path(record["source_image"]).stem


def load_records(condition: str, domain: str) -> dict[str, dict[str, Any]]:
    base = FEATURE_ROOT / condition / domain
    manifest_path = base / "feature_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(manifest_path)
    manifest = read_json(manifest_path)
    records = {}
    for record in manifest["records"]:
        pair_id = pair_id_from_source(record)
        record = dict(record)
        record["_feature_base"] = str(base)
        records[pair_id] = record
    return records


def feature(record: dict[str, Any], key: str) -> np.ndarray:
    base = Path(record["_feature_base"])
    data = np.load(base / record["feature_file"], allow_pickle=True)
    return np.asarray(data[key], dtype=np.float64)


def l2(arr: np.ndarray) -> float:
    return float(np.linalg.norm(arr))


def cosine_distance(left: np.ndarray, right: np.ndarray) -> float:
    x = left.reshape(-1)
    y = right.reshape(-1)
    denom = float(np.linalg.norm(x) * np.linalg.norm(y))
    if denom <= 1e-12:
        return 0.0
    return float(1.0 - np.dot(x, y) / denom)


def token_l2(left: np.ndarray, right: np.ndarray) -> float:
    diff = np.asarray(left - right, dtype=np.float64)
    if diff.ndim >= 3:
        diff = diff.reshape(-1, diff.shape[-1])
    return float(np.mean(np.linalg.norm(diff, axis=-1)))


def summarize(values: list[float]) -> float:
    arr = np.asarray(values, dtype=np.float64)
    return float(np.mean(arr)) if arr.size else float("nan")


def main() -> int:
    if not FEATURE_ROOT.is_dir():
        write_json(
            PHASE6 / "full_forward_summary.json",
            {
                "feature_root": str(FEATURE_ROOT),
                "condition_count": 0,
                "status": "FULL_FORWARD_REQUIRED",
                "reason": "Feature root does not exist. Run scripts/run_full_forward_feature_extraction.sh on a CUDA-capable machine first.",
            },
        )
        print(json.dumps({"status": "FULL_FORWARD_REQUIRED", "feature_root": str(FEATURE_ROOT)}, indent=2))
        return 0
    obs_rows = read_csv(OBS_TABLE) if OBS_TABLE.is_file() else []
    obs_by_condition = {row["condition"]: row for row in obs_rows}
    pair_manifest = read_json(PAIR_MANIFEST)
    pair_ids = [pair["pair_id"] for pair in pair_manifest["pairs"]]
    phase_by_pair = {pair["pair_id"]: pair.get("planner_phase", "") for pair in pair_manifest["pairs"]}

    conditions = sorted(path.name for path in FEATURE_ROOT.iterdir() if path.is_dir())
    frame_rows: list[dict[str, Any]] = []
    summary_rows: list[dict[str, Any]] = []

    for condition in conditions:
        real = load_records(condition, "real")
        sim = load_records(condition, "sim")
        common = [pair_id for pair_id in pair_ids if pair_id in real and pair_id in sim]
        if not common:
            continue
        values: dict[str, list[float]] = {
            "vision_pooled_cosine": [],
            "vision_pooled_l2": [],
            "vision_token_l2": [],
            "projector_pooled_cosine": [],
            "projector_pooled_l2": [],
            "projector_token_l2": [],
            "hidden_cosine": [],
            "hidden_l2": [],
            "action_chunk_mean_l2": [],
            "action_chunk_max_l2": [],
            "action_first_l2": [],
            "translation_l2": [],
            "rotation_l2": [],
            "gripper_gap": [],
        }
        for pair_id in common:
            rr = real[pair_id]
            ss = sim[pair_id]
            real_actions = action_chunk(rr)
            sim_actions = action_chunk(ss)
            n = min(len(real_actions), len(sim_actions))
            delta_action = real_actions[:n] - sim_actions[:n]
            real_vision = feature(rr, "vision_backbone.output")
            sim_vision = feature(ss, "vision_backbone.output")
            real_proj = feature(rr, "projector.output")
            sim_proj = feature(ss, "projector.output")
            real_hidden = feature(rr, "action_hidden_states.input")
            sim_hidden = feature(ss, "action_hidden_states.input")
            row = {
                "pair_id": pair_id,
                "condition": condition,
                "planner_phase": phase_by_pair.get(pair_id, ""),
                "vision_pooled_cosine": cosine_distance(real_vision.mean(axis=1), sim_vision.mean(axis=1)),
                "vision_pooled_l2": l2(real_vision.mean(axis=1) - sim_vision.mean(axis=1)),
                "vision_token_l2": token_l2(real_vision, sim_vision),
                "projector_pooled_cosine": cosine_distance(real_proj.mean(axis=1), sim_proj.mean(axis=1)),
                "projector_pooled_l2": l2(real_proj.mean(axis=1) - sim_proj.mean(axis=1)),
                "projector_token_l2": token_l2(real_proj, sim_proj),
                "hidden_cosine": cosine_distance(real_hidden, sim_hidden),
                "hidden_l2": l2(real_hidden - sim_hidden),
                "action_chunk_mean_l2": float(np.mean(np.linalg.norm(delta_action, axis=1))),
                "action_chunk_max_l2": float(np.max(np.linalg.norm(delta_action, axis=1))),
                "action_first_l2": l2(delta_action[0]),
                "translation_l2": l2(delta_action[0, :3]),
                "rotation_l2": l2(delta_action[0, 3:6]),
                "gripper_gap": float(abs(delta_action[0, 6])),
            }
            for key in values:
                values[key].append(float(row[key]))
            frame_rows.append(row)
        obs = obs_by_condition.get(condition, {})
        action_gap = summarize(values["action_chunk_mean_l2"])
        baseline_gap = None
        summary_rows.append(
            {
                "factor": obs.get("factor", ""),
                "condition": condition,
                "count": len(common),
                "observation_rgb_l2_mean": obs.get("observation_rgb_l2_mean", ""),
                "observation_mse": obs.get("observation_mse", ""),
                "observation_psnr": obs.get("observation_psnr", ""),
                "observation_ssim_global": obs.get("observation_ssim_global", ""),
                "vision_gap": summarize(values["vision_pooled_l2"]),
                "projector_gap": summarize(values["projector_pooled_l2"]),
                "hidden_gap": summarize(values["hidden_l2"]),
                "sensitive_energy": "",
                "null_energy": "",
                "sensitive_ratio": "",
                "action_gap": action_gap,
                "chunk_max_l2": summarize(values["action_chunk_max_l2"]),
                "first_action_l2": summarize(values["action_first_l2"]),
                "translation_gap": summarize(values["translation_l2"]),
                "rotation_gap": summarize(values["rotation_l2"]),
                "gripper_gap": summarize(values["gripper_gap"]),
                "policy_impact_score": "",
                "policy_efficiency": "",
                "status": "VERIFIED_FULL_FORWARD" if len(common) == len(pair_ids) else "PARTIAL_FULL_FORWARD",
            }
        )

    baseline = next((row for row in summary_rows if row["condition"] == "P0_current_paired_image"), None)
    if baseline:
        base_action = float(baseline["action_gap"])
        base_hidden = float(baseline["hidden_gap"])
        base_vision = float(baseline["vision_gap"])
        for row in summary_rows:
            action = float(row["action_gap"])
            hidden = float(row["hidden_gap"])
            vision = float(row["vision_gap"])
            row["policy_impact_score"] = base_action - action
            row["action_gap_reduction_pct"] = (base_action - action) / base_action * 100.0 if base_action else math.nan
            row["hidden_gap_reduction_pct"] = (base_hidden - hidden) / base_hidden * 100.0 if base_hidden else math.nan
            row["vision_gap_reduction_pct"] = (base_vision - vision) / base_vision * 100.0 if base_vision else math.nan
            denom = base_hidden - hidden
            row["policy_efficiency"] = (base_action - action) / denom if abs(denom) > 1e-12 else ""

    PHASE6.mkdir(parents=True, exist_ok=True)
    frame_path = PHASE6 / "policy_sensitive_attribution_frame_metrics.csv"
    if frame_rows:
        with frame_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(frame_rows[0].keys()))
            writer.writeheader()
            writer.writerows(frame_rows)
    summary_path = PHASE6 / "environment_factor_table.csv"
    if summary_rows:
        with summary_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0].keys()))
            writer.writeheader()
            writer.writerows(summary_rows)
        table_copy = PHASE6 / "tables" / "environment_factor_table.csv"
        table_copy.parent.mkdir(parents=True, exist_ok=True)
        table_copy.write_text(summary_path.read_text(encoding="utf-8"), encoding="utf-8")
    write_json(
        PHASE6 / "full_forward_summary.json",
        {
            "feature_root": str(FEATURE_ROOT),
            "condition_count": len(summary_rows),
            "pair_count_expected": len(pair_ids),
            "summary_csv": str(summary_path),
            "frame_metrics_csv": str(frame_path),
            "status": "VERIFIED_FULL_FORWARD" if summary_rows else "NO_FEATURES_FOUND",
            "notes": [
                "Sensitive/null energy is not filled here because Phase5 sensitive basis was not serialized.",
                "Action metrics are valid only when condition features were extracted with the fixed oftplus_h5_vision checkpoint.",
            ],
        },
    )
    print(json.dumps({"summary_csv": str(summary_path), "rows": len(summary_rows)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
