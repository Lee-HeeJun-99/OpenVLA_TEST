#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np


ACTION_DIM = 7
CHUNK = 5
GRIPPER_THRESHOLD = 0.5


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


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def pair_id_from_source(source_image: str) -> str:
    return Path(source_image).stem


def episode_id(pair_id: str) -> str:
    match = re.match(r"(episode_\d{6})_", pair_id)
    if not match:
        raise ValueError(f"Cannot parse episode from pair_id={pair_id}")
    return match.group(1)


def step_index(pair_id: str) -> int:
    match = re.search(r"_(\d+)$", pair_id)
    if not match:
        raise ValueError(f"Cannot parse step from pair_id={pair_id}")
    return int(match.group(1))


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions")
    if actions is None:
        action = response.get("action")
        actions = [action] if action is not None else None
    arr = np.asarray(actions, dtype=np.float64)
    if arr.shape != (CHUNK, ACTION_DIM):
        raise ValueError(f"Unexpected action shape {arr.shape} for {record.get('source_image')}")
    return arr


def load_records(manifest_path: Path) -> dict[str, dict[str, Any]]:
    manifest = read_json(manifest_path)
    records: dict[str, dict[str, Any]] = {}
    for record in manifest.get("records", []):
        records[pair_id_from_source(record["source_image"])] = record
    return records


def cosine_distance(left: np.ndarray, right: np.ndarray) -> float:
    x = np.asarray(left, dtype=np.float64).reshape(-1)
    y = np.asarray(right, dtype=np.float64).reshape(-1)
    denom = float(np.linalg.norm(x) * np.linalg.norm(y))
    if denom <= 1.0e-12:
        return 0.0
    return float(1.0 - np.dot(x, y) / denom)


def huber(delta: np.ndarray, threshold: float) -> float:
    d = np.abs(np.asarray(delta, dtype=np.float64).reshape(-1))
    vals = np.where(d <= threshold, 0.5 * d * d, threshold * (d - 0.5 * threshold))
    return float(np.mean(vals))


def transition_index(values: np.ndarray, threshold: float = GRIPPER_THRESHOLD) -> int:
    closed = np.asarray(values, dtype=np.float64) >= threshold
    indices = np.where(closed)[0]
    return int(indices[0]) if indices.size else -1


def metric_row(real: np.ndarray, sim: np.ndarray) -> dict[str, float]:
    delta = real - sim
    abs_delta = np.abs(delta)
    l2_steps = np.linalg.norm(delta, axis=1)
    l1_steps = np.sum(abs_delta, axis=1)
    rmse_steps = np.sqrt(np.mean(delta * delta, axis=1))
    real_closed = real[:, 6] >= GRIPPER_THRESHOLD
    sim_closed = sim[:, 6] >= GRIPPER_THRESHOLD
    real_transition = transition_index(real[:, 6])
    sim_transition = transition_index(sim[:, 6])
    out: dict[str, float] = {
        "action_l1_chunk_mean": float(np.mean(l1_steps)),
        "action_l1_chunk_max": float(np.max(l1_steps)),
        "action_mae_chunk_mean": float(np.mean(abs_delta)),
        "action_l2_chunk_mean": float(np.mean(l2_steps)),
        "action_l2_chunk_max": float(np.max(l2_steps)),
        "action_rmse_chunk_mean": float(np.mean(rmse_steps)),
        "action_huber0.01_chunk_mean": huber(delta, 0.01),
        "action_huber0.05_chunk_mean": huber(delta, 0.05),
        "action_huber0.1_chunk_mean": huber(delta, 0.1),
        "action_cosine_chunk_mean": float(np.mean([cosine_distance(real[i], sim[i]) for i in range(real.shape[0])])),
        "first_action_l2": float(np.linalg.norm(delta[0])),
        "translation_l2_chunk_mean": float(np.mean(np.linalg.norm(delta[:, :3], axis=1))),
        "translation_mae_chunk_mean": float(np.mean(abs_delta[:, :3])),
        "rotation_l2_chunk_mean": float(np.mean(np.linalg.norm(delta[:, 3:6], axis=1))),
        "rotation_mae_chunk_mean": float(np.mean(abs_delta[:, 3:6])),
        "gripper_abs_chunk_mean": float(np.mean(abs_delta[:, 6])),
        "gripper_binary_disagreement_rate": float(np.mean(real_closed != sim_closed)),
        "real_gripper_transition_index": float(real_transition),
        "sim_gripper_transition_index": float(sim_transition),
        "transition_delta_sim_minus_real": float(sim_transition - real_transition),
    }
    for index in range(ACTION_DIM):
        out[f"dim{index}_mae"] = float(np.mean(abs_delta[:, index]))
    return out


def summarize(rows: list[dict[str, Any]], group_keys: list[str], metrics: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[key] for key in group_keys), []).append(row)
    output: list[dict[str, Any]] = []
    for key, items in sorted(grouped.items(), key=lambda item: item[0]):
        result = {name: value for name, value in zip(group_keys, key)}
        result["count"] = len(items)
        for metric in metrics:
            vals = np.asarray([float(item[metric]) for item in items], dtype=np.float64)
            result[f"{metric}_mean"] = float(np.mean(vals))
            result[f"{metric}_std"] = float(np.std(vals))
            result[f"{metric}_median"] = float(np.median(vals))
            result[f"{metric}_p90"] = float(np.percentile(vals, 90))
        output.append(result)
    return output


def phase_for_pair(pair_id: str, phase_lookup: dict[str, str]) -> str:
    return phase_lookup.get(pair_id, "unknown")


def load_phase_lookup(root: Path) -> dict[str, str]:
    path = root / "outputs" / "token_distribution_analysis" / "5_episodes" / "episode_phase_summary" / "frame_metrics_enriched.csv"
    if not path.is_file():
        return {}
    lookup: dict[str, str] = {}
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            pid = row.get("pair_id") or row.get("frame_id") or ""
            phase = row.get("phase") or row.get("planner_phase") or ""
            if pid and phase:
                lookup[pid] = phase
    return lookup


def collect_condition(feature_root: Path, condition: str, root: Path, group: str, phase_lookup: dict[str, str]) -> list[dict[str, Any]]:
    real_path = feature_root / condition / "real" / "feature_manifest.json"
    sim_path = feature_root / condition / "sim" / "feature_manifest.json"
    if not real_path.is_file() or not sim_path.is_file():
        return []
    real_records = load_records(real_path)
    sim_records = load_records(sim_path)
    common = sorted(set(real_records) & set(sim_records), key=lambda pid: (episode_id(pid), step_index(pid)))
    rows: list[dict[str, Any]] = []
    for pid in common:
        real = action_chunk(real_records[pid])
        sim = action_chunk(sim_records[pid])
        row: dict[str, Any] = {
            "group": group,
            "condition": condition,
            "pair_id": pid,
            "episode_id": episode_id(pid),
            "step_index": step_index(pid),
            "phase": phase_for_pair(pid, phase_lookup),
        }
        row.update(metric_row(real, sim))
        rows.append(row)
    return rows


def update_priority2_reports(out: Path, camera_rows: list[dict[str, Any]], letterbox_rows: list[dict[str, Any]]) -> None:
    report = out / "full_forward_priority2_report.md"
    all_rows = camera_rows + letterbox_rows
    best = sorted(
        summarize(all_rows, ["group", "condition"], ["action_l2_chunk_mean"]),
        key=lambda row: float(row["action_l2_chunk_mean_mean"]),
    )
    lines = [
        "# Priority2 Full-Forward Postprocess Report",
        "",
        "Status: `VERIFIED_FULL_FORWARD_POSTPROCESS`",
        "",
        f"- Camera frame rows: {len(camera_rows)}",
        f"- Letterbox frame rows: {len(letterbox_rows)}",
        f"- Total frame rows: {len(all_rows)}",
        "",
        "## Best Conditions By Action L2",
        "",
    ]
    for row in best[:10]:
        lines.append(
            f"- {row['group']} / {row['condition']}: "
            f"{float(row['action_l2_chunk_mean_mean']):.6f} "
            f"(N={row['count']})"
        )
    lines.extend(
        [
            "",
            "These numbers are image-to-final-action full-forward results from the saved feature manifests.",
            "They remain offline action-disagreement evidence, not real robot performance evidence.",
            "",
        ]
    )
    write_text(report, "\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle-root", type=Path, required=True)
    parser.add_argument("--priority2-dir", type=Path, required=True)
    args = parser.parse_args()

    root = args.bundle_root.expanduser().resolve()
    out = args.priority2_dir.expanduser().resolve()
    phase_lookup = load_phase_lookup(root)

    camera_root = out / "02_matched_camera_shift" / "full_forward_features"
    letterbox_root = out / "03_letterbox_ablation" / "full_forward_features"
    camera_conditions = sorted([p.name for p in camera_root.iterdir() if (p / "real" / "feature_manifest.json").is_file()]) if camera_root.is_dir() else []
    letterbox_conditions = sorted([p.name for p in letterbox_root.iterdir() if (p / "real" / "feature_manifest.json").is_file()]) if letterbox_root.is_dir() else []

    camera_rows: list[dict[str, Any]] = []
    for condition in camera_conditions:
        camera_rows.extend(collect_condition(camera_root, condition, root, "matched_camera_shift", phase_lookup))
    letterbox_rows: list[dict[str, Any]] = []
    for condition in letterbox_conditions:
        letterbox_rows.extend(collect_condition(letterbox_root, condition, root, "letterbox_ablation", phase_lookup))

    metrics = [
        "action_l1_chunk_mean",
        "action_mae_chunk_mean",
        "action_l2_chunk_mean",
        "action_rmse_chunk_mean",
        "action_huber0.05_chunk_mean",
        "action_cosine_chunk_mean",
        "first_action_l2",
        "translation_l2_chunk_mean",
        "rotation_l2_chunk_mean",
        "gripper_abs_chunk_mean",
        "gripper_binary_disagreement_rate",
    ]

    write_csv(out / "02_matched_camera_shift" / "full_forward_frame_metrics.csv", camera_rows)
    write_csv(out / "02_matched_camera_shift" / "full_forward_episode_metrics.csv", summarize(camera_rows, ["condition", "episode_id"], metrics) if camera_rows else [])
    write_csv(out / "02_matched_camera_shift" / "full_forward_phase_metrics.csv", summarize(camera_rows, ["condition", "phase"], metrics) if camera_rows else [])

    write_csv(out / "03_letterbox_ablation" / "full_forward_frame_metrics.csv", letterbox_rows)
    write_csv(out / "03_letterbox_ablation" / "full_forward_episode_metrics.csv", summarize(letterbox_rows, ["condition", "episode_id"], metrics) if letterbox_rows else [])
    write_csv(out / "03_letterbox_ablation" / "full_forward_phase_metrics.csv", summarize(letterbox_rows, ["condition", "phase"], metrics) if letterbox_rows else [])

    combined = camera_rows + letterbox_rows
    write_csv(out / "full_forward_condition_metrics.csv", summarize(combined, ["group", "condition"], metrics) if combined else [])
    write_json(
        out / "full_forward_postprocess_status.json",
        {
            "status": "VERIFIED_FULL_FORWARD_POSTPROCESS" if combined else "NO_FULL_FORWARD_FEATURES_FOUND",
            "camera_conditions": camera_conditions,
            "letterbox_conditions": letterbox_conditions,
            "camera_frame_rows": len(camera_rows),
            "letterbox_frame_rows": len(letterbox_rows),
            "total_frame_rows": len(combined),
            "policy_variant": "oftplus_h5_vision",
            "checkpoint_step": 28560,
            "priority3_started": False,
            "priority4_started": False,
        },
    )
    update_priority2_reports(out, camera_rows, letterbox_rows)
    print(json.dumps({"status": "ok", "camera_rows": len(camera_rows), "letterbox_rows": len(letterbox_rows)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
