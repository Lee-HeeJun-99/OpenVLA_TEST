#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Any

import numpy as np
import torch


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
DEFAULT_ANALYSIS_ROOT = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes"
DEFAULT_OUTPUT_DIR = ROOT / "lhj" / "phase1_action_gap" / "policy_relevant_progress_shift"
DEFAULT_CHECKPOINT = ROOT / "runtime_state" / "oft_mixed480_step28560_merged"
OFT_REPO = ROOT / "runtime" / "openvla-oft"
DATASET_KEY = "a0509_sim_cube_pick"
ACTION_DIM = 7
CHUNK = 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--analysis-root", type=Path, default=DEFAULT_ANALYSIS_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--window", type=int, default=11)
    parser.add_argument(
        "--skip-hidden-action-head",
        action="store_true",
        help="Only evaluate action_head.output shift; do not load action head.",
    )
    return parser.parse_args()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


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
    match = re.match(r"(episode_\d{6})_", str(frame))
    if not match:
        raise ValueError(f"Could not parse episode id from {frame!r}")
    return match.group(1)


def progress_from_frame(frame: str) -> int:
    match = re.search(r"(\d+)$", str(frame))
    if not match:
        raise ValueError(f"Could not parse progress from {frame!r}")
    return int(match.group(1))


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions")
    if actions is None:
        actions = [response.get("action")]
    arr = np.asarray(actions, dtype=np.float64)
    if arr.shape != (CHUNK, ACTION_DIM):
        raise ValueError(f"Unexpected action shape for {record.get('source_image')}: {arr.shape}")
    return arr


def load_npz_array(record: dict[str, Any], key: str) -> np.ndarray:
    with np.load(record["_feature_path"], allow_pickle=True) as data:
        return np.asarray(data[key], dtype=np.float32)


def nearest_shift(
    train_progress: np.ndarray,
    train_shift: np.ndarray,
    test_progress_value: int,
    window: int,
) -> np.ndarray:
    half = window // 2
    near = np.where(np.abs(train_progress - test_progress_value) <= half)[0]
    if near.size == 0:
        near = np.asarray([int(np.argmin(np.abs(train_progress - test_progress_value)))])
    return train_shift[near].mean(axis=0)


def unnormalize_actions(normalized_actions: np.ndarray, stats: dict[str, Any]) -> np.ndarray:
    norm = np.asarray(normalized_actions, dtype=np.float64)
    mask = np.asarray(stats.get("mask", np.ones_like(stats["q01"], dtype=bool)), dtype=bool)
    high = np.asarray(stats["q99"], dtype=np.float64)
    low = np.asarray(stats["q01"], dtype=np.float64)
    return np.where(mask, 0.5 * (norm + 1.0) * (high - low + 1e-8) + low, norm)


def summarize(values: list[float]) -> dict[str, float | int]:
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "min": float(arr.min()),
        "p50": float(np.percentile(arr, 50)),
        "p90": float(np.percentile(arr, 90)),
        "max": float(arr.max()),
    }


def chunk_metrics(left: np.ndarray, right: np.ndarray) -> dict[str, float]:
    delta = np.asarray(left, dtype=np.float64) - np.asarray(right, dtype=np.float64)
    first = delta[0]
    chunk_l2 = np.linalg.norm(delta, axis=1)
    return {
        "first_l2": float(np.linalg.norm(first)),
        "first_translation_l2": float(np.linalg.norm(first[:3])),
        "first_rotation_l2": float(np.linalg.norm(first[3:6])),
        "first_gripper_abs": float(abs(first[6])),
        "chunk_mean_l2": float(chunk_l2.mean()),
        "chunk_max_l2": float(chunk_l2.max()),
    }


def load_action_head(checkpoint: Path):
    os.environ.setdefault("A0509_ACTION_CHUNK_SIZE", str(CHUNK))
    if str(OFT_REPO) not in sys.path:
        sys.path.insert(0, str(OFT_REPO))
    from prismatic.models.action_heads import L1RegressionActionHead

    training_config_path = checkpoint / "a0509_training_config.json"
    training_config = (
        read_json(training_config_path) if training_config_path.is_file() else {}
    )
    action_head = L1RegressionActionHead(
        input_dim=4096,
        hidden_dim=4096,
        action_dim=ACTION_DIM,
        bounded_gripper=bool(training_config.get("bounded_gripper", False)),
        gripper_loss_weight=float(training_config.get("gripper_loss_weight", 3.0)),
    )
    matches = sorted(checkpoint.glob("action_head*checkpoint*.pt"))
    if len(matches) != 1:
        raise FileNotFoundError(f"Expected one action_head checkpoint in {checkpoint}, found {len(matches)}")
    state_dict = torch.load(matches[0], map_location="cpu", weights_only=True)
    state_dict = {
        (key[7:] if key.startswith("module.") else key): value
        for key, value in state_dict.items()
    }
    action_head.load_state_dict(state_dict)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    dtype = torch.bfloat16 if device.type == "cuda" else torch.float32
    action_head = action_head.to(device=device, dtype=dtype).eval()
    return action_head, device, dtype, str(matches[0])


def predict_norm_from_hidden(
    action_head: torch.nn.Module,
    device: torch.device,
    dtype: torch.dtype,
    hidden: np.ndarray,
) -> np.ndarray:
    with torch.inference_mode():
        tensor = torch.as_tensor(hidden, device=device, dtype=dtype)
        output = action_head.predict_action(tensor)
        output = output.reshape(CHUNK, ACTION_DIM).float().cpu().numpy()
    return output.astype(np.float64)


def main() -> int:
    args = parse_args()
    if args.window < 3 or args.window % 2 == 0:
        raise ValueError("--window must be odd and >= 3")

    analysis_root = args.analysis_root.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    checkpoint = args.checkpoint.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    real_records = records_by_frame(analysis_root / "features" / "real" / "feature_manifest.json")
    sim_records = records_by_frame(analysis_root / "features" / "sim" / "feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))
    episode_ids = sorted({episode_id_from_frame(frame) for frame in frames})

    action_stats = read_json(checkpoint / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head = None
    device = torch.device("cpu")
    dtype = torch.float32
    action_head_checkpoint = None
    if not args.skip_hidden_action_head:
        action_head, device, dtype, action_head_checkpoint = load_action_head(checkpoint)

    real_actions = {frame: action_chunk(real_records[frame]) for frame in frames}
    sim_actions = {frame: action_chunk(sim_records[frame]) for frame in frames}
    real_norm = {frame: load_npz_array(real_records[frame], "action_head.output").reshape(CHUNK, ACTION_DIM) for frame in frames}
    sim_norm = {frame: load_npz_array(sim_records[frame], "action_head.output").reshape(CHUNK, ACTION_DIM) for frame in frames}

    real_hidden = {}
    sim_hidden = {}
    if action_head is not None:
        real_hidden = {frame: load_npz_array(real_records[frame], "action_hidden_states.input") for frame in frames}
        sim_hidden = {frame: load_npz_array(sim_records[frame], "action_hidden_states.input") for frame in frames}

    rows: list[dict[str, Any]] = []
    recompute_error: list[float] = []

    for test_episode in episode_ids:
        train_frames = [frame for frame in frames if episode_id_from_frame(frame) != test_episode]
        test_frames = [frame for frame in frames if episode_id_from_frame(frame) == test_episode]
        train_progress = np.asarray([progress_from_frame(frame) for frame in train_frames], dtype=np.int64)

        train_norm_shift = np.stack([(sim_norm[frame] - real_norm[frame]).reshape(-1) for frame in train_frames])
        train_hidden_shift = None
        if action_head is not None:
            train_hidden_shift = np.stack([(sim_hidden[frame] - real_hidden[frame]).reshape(-1) for frame in train_frames])

        for frame in test_frames:
            progress = progress_from_frame(frame)
            raw_metrics = chunk_metrics(real_actions[frame], sim_actions[frame])

            norm_shift = nearest_shift(train_progress, train_norm_shift, progress, args.window).reshape(CHUNK, ACTION_DIM)
            corrected_norm_action = unnormalize_actions(real_norm[frame] + norm_shift, action_stats)
            norm_metrics = chunk_metrics(corrected_norm_action, sim_actions[frame])

            row: dict[str, Any] = {
                "frame": frame,
                "episode_id": test_episode,
                "progress": progress,
                "window": args.window,
                "raw_first_l2": raw_metrics["first_l2"],
                "raw_first_translation_l2": raw_metrics["first_translation_l2"],
                "raw_first_rotation_l2": raw_metrics["first_rotation_l2"],
                "raw_first_gripper_abs": raw_metrics["first_gripper_abs"],
                "raw_chunk_mean_l2": raw_metrics["chunk_mean_l2"],
                "raw_chunk_max_l2": raw_metrics["chunk_max_l2"],
                "norm_output_shift_first_l2": norm_metrics["first_l2"],
                "norm_output_shift_first_translation_l2": norm_metrics["first_translation_l2"],
                "norm_output_shift_first_rotation_l2": norm_metrics["first_rotation_l2"],
                "norm_output_shift_first_gripper_abs": norm_metrics["first_gripper_abs"],
                "norm_output_shift_chunk_mean_l2": norm_metrics["chunk_mean_l2"],
                "norm_output_shift_chunk_max_l2": norm_metrics["chunk_max_l2"],
                "norm_output_shift_norm": float(np.linalg.norm(norm_shift)),
            }

            if action_head is not None and train_hidden_shift is not None:
                hidden_shift = nearest_shift(
                    train_progress,
                    train_hidden_shift,
                    progress,
                    args.window,
                ).reshape(real_hidden[frame].shape)
                corrected_hidden = real_hidden[frame] + hidden_shift
                corrected_hidden_norm = predict_norm_from_hidden(action_head, device, dtype, corrected_hidden)
                corrected_hidden_action = unnormalize_actions(corrected_hidden_norm, action_stats)
                hidden_metrics = chunk_metrics(corrected_hidden_action, sim_actions[frame])
                row.update(
                    {
                        "hidden_shift_first_l2": hidden_metrics["first_l2"],
                        "hidden_shift_first_translation_l2": hidden_metrics["first_translation_l2"],
                        "hidden_shift_first_rotation_l2": hidden_metrics["first_rotation_l2"],
                        "hidden_shift_first_gripper_abs": hidden_metrics["first_gripper_abs"],
                        "hidden_shift_chunk_mean_l2": hidden_metrics["chunk_mean_l2"],
                        "hidden_shift_chunk_max_l2": hidden_metrics["chunk_max_l2"],
                        "hidden_shift_norm": float(np.linalg.norm(hidden_shift)),
                    }
                )
                recomputed_real_norm = predict_norm_from_hidden(action_head, device, dtype, real_hidden[frame])
                recomputed_real_action = unnormalize_actions(recomputed_real_norm, action_stats)
                recompute_error.append(float(np.max(np.abs(recomputed_real_action - real_actions[frame]))))
            rows.append(row)

    fieldnames = list(rows[0].keys())
    with (output_dir / "policy_relevant_progress_shift_frame_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    metric_keys = [key for key in fieldnames if key.endswith("_l2") or key.endswith("_abs")]
    summary = {
        "method": "leave-one-episode-out real_to_sim progress shift evaluated on final actions",
        "analysis_root": str(analysis_root),
        "checkpoint": str(checkpoint),
        "output_dir": str(output_dir),
        "window": args.window,
        "paired_frame_count": len(frames),
        "episode_ids": episode_ids,
        "action_head_checkpoint": action_head_checkpoint,
        "action_head_device": str(device),
        "action_head_dtype": str(dtype),
        "metrics": {key: summarize([float(row[key]) for row in rows if key in row]) for key in metric_keys},
        "improvements": {},
        "recompute_error_vs_manifest_action": summarize(recompute_error) if recompute_error else None,
        "notes": [
            "raw_* compares final unnormalized Real response.actions with Sim response.actions.",
            "norm_output_shift_* applies LOO progress shift directly to saved normalized action_head.output, then uses dataset q01/q99 unnormalization.",
            "hidden_shift_* applies LOO progress shift to action_hidden_states.input, reruns the saved action head, then uses dataset q01/q99 unnormalization.",
            "Test episode Sim features are not used to estimate the shift for that test episode.",
            "This is an offline action-gap test, not a real robot performance result.",
        ],
    }
    for target in ("norm_output_shift", "hidden_shift"):
        for suffix in ("first_l2", "first_translation_l2", "first_rotation_l2", "first_gripper_abs", "chunk_mean_l2", "chunk_max_l2"):
            key = f"{target}_{suffix}"
            raw_key = f"raw_{suffix}"
            if key in summary["metrics"]:
                raw_values = np.asarray([float(row[raw_key]) for row in rows], dtype=np.float64)
                shifted_values = np.asarray([float(row[key]) for row in rows if key in row], dtype=np.float64)
                if raw_values.size == shifted_values.size:
                    summary["improvements"][f"{target}_{suffix}"] = {
                        "raw_mean": float(raw_values.mean()),
                        "shifted_mean": float(shifted_values.mean()),
                        "mean_delta_raw_minus_shifted": float(raw_values.mean() - shifted_values.mean()),
                        "relative_delta": float((raw_values.mean() - shifted_values.mean()) / raw_values.mean())
                        if abs(raw_values.mean()) > 1e-12
                        else None,
                    }
    write_json(output_dir / "policy_relevant_progress_shift_summary.json", summary)
    print(json.dumps(summary["improvements"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
