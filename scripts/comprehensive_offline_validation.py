#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
ANALYSIS_ROOT = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes"
LHJ = ROOT / "lhj"
OUT = LHJ / "phase2_full_validation"
ROS_RUNTIME = Path("/home/ubuntu/robot_ws/src/openvla_doosan_runtime/config/runtime_oft.yaml")
BUNDLE_CHECKPOINT = ROOT / "runtime_state" / "oft_mixed480_step28560_merged"


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


def frame_from_source(path: str) -> str:
    return Path(path).stem


def flatten_numeric(obj: Any, prefix: str = "") -> dict[str, float]:
    out: dict[str, float] = {}
    if isinstance(obj, dict):
        for key, value in obj.items():
            new_prefix = f"{prefix}.{key}" if prefix else str(key)
            out.update(flatten_numeric(value, new_prefix))
    elif isinstance(obj, (int, float)) and not isinstance(obj, bool):
        value = float(obj)
        if math.isfinite(value):
            out[prefix] = value
    return out


def compare_json_numbers(left_path: Path, right_path: Path, rel_tol: float = 1e-10, abs_tol: float = 1e-10) -> dict[str, Any]:
    left = flatten_numeric(read_json(left_path))
    right = flatten_numeric(read_json(right_path))
    keys = sorted(set(left) | set(right))
    mismatches = []
    max_abs = 0.0
    for key in keys:
        if key not in left or key not in right:
            mismatches.append({"key": key, "reason": "missing", "left": left.get(key), "right": right.get(key)})
            continue
        diff = abs(left[key] - right[key])
        max_abs = max(max_abs, diff)
        if diff > abs_tol + rel_tol * abs(right[key]):
            mismatches.append({"key": key, "reason": "value", "left": left[key], "right": right[key], "abs_diff": diff})
    return {
        "left": str(left_path),
        "right": str(right_path),
        "left_numeric_count": len(left),
        "right_numeric_count": len(right),
        "max_abs_diff": max_abs,
        "mismatch_count": len(mismatches),
        "mismatches_first_20": mismatches[:20],
        "status": "PASS" if not mismatches else "FAIL",
    }


def manifest_integrity(domain: str) -> dict[str, Any]:
    manifest_path = ANALYSIS_ROOT / "features" / domain / "feature_manifest.json"
    manifest = read_json(manifest_path)
    records = manifest["records"]
    frames = [frame_from_source(record["source_image"]) for record in records]
    feature_paths = [(manifest_path.parent / record["feature_file"]) for record in records]
    image_paths = [Path(record["source_image"]) for record in records]
    missing_features = [str(path) for path in feature_paths if not path.is_file()]
    missing_images = [str(path) for path in image_paths if not path.is_file()]
    duplicate_frames = sorted({frame for frame in frames if frames.count(frame) > 1})

    expected_keys = {
        "vision_backbone.output": (1, 256, 2176),
        "vision_backbone.input": (1, 6, 224, 224),
        "projector.output": (1, 256, 4096),
        "projector.input": (1, 256, 2176),
        "action_hidden_states.input": (1, 35, 4096),
        "action_head.output": (1, 5, 7),
        "metadata_json": None,
    }
    shape_errors = []
    checked_npz = 0
    for path in feature_paths:
        if not path.is_file():
            continue
        checked_npz += 1
        with np.load(path, allow_pickle=True) as data:
            keys = set(data.files)
            missing = sorted(set(expected_keys) - keys)
            extra = sorted(keys - set(expected_keys))
            if missing or extra:
                shape_errors.append({"path": str(path), "missing_keys": missing, "extra_keys": extra})
            for key, expected_shape in expected_keys.items():
                if expected_shape is None or key not in keys:
                    continue
                arr = data[key]
                if tuple(arr.shape) != expected_shape:
                    shape_errors.append(
                        {
                            "path": str(path),
                            "key": key,
                            "expected_shape": list(expected_shape),
                            "actual_shape": list(arr.shape),
                        }
                    )

    image_sizes: dict[str, int] = {}
    image_mode_counts: dict[str, int] = {}
    for path in image_paths:
        if not path.is_file():
            continue
        with Image.open(path) as image:
            image_sizes[f"{image.width}x{image.height}"] = image_sizes.get(f"{image.width}x{image.height}", 0) + 1
            image_mode_counts[image.mode] = image_mode_counts.get(image.mode, 0) + 1

    instructions = sorted({record.get("instruction") for record in records})
    variants = sorted({record.get("response", {}).get("variant") for record in records})
    chunk_sizes = sorted({record.get("response", {}).get("chunk_size") for record in records})
    action_shape_errors = []
    for record in records:
        actions = record.get("response", {}).get("actions")
        arr = np.asarray(actions, dtype=np.float64)
        if arr.shape != (5, 7):
            action_shape_errors.append({"frame": frame_from_source(record["source_image"]), "shape": list(arr.shape)})

    status = "PASS"
    if missing_features or missing_images or duplicate_frames or shape_errors or action_shape_errors:
        status = "FAIL"

    return {
        "domain": domain,
        "manifest": str(manifest_path),
        "record_count": len(records),
        "unique_frame_count": len(set(frames)),
        "duplicate_frames": duplicate_frames,
        "missing_feature_count": len(missing_features),
        "missing_image_count": len(missing_images),
        "checked_npz_count": checked_npz,
        "shape_error_count": len(shape_errors),
        "shape_errors_first_20": shape_errors[:20],
        "image_sizes": image_sizes,
        "image_modes": image_mode_counts,
        "instructions": instructions,
        "variants": variants,
        "chunk_sizes": chunk_sizes,
        "action_shape_error_count": len(action_shape_errors),
        "action_shape_errors_first_20": action_shape_errors[:20],
        "status": status,
    }


def paired_integrity() -> dict[str, Any]:
    paired_path = ANALYSIS_ROOT / "paired_inputs" / "paired_manifest.json"
    paired = read_json(paired_path)
    records = paired.get("pairs", paired.get("records", []))
    if not records:
        records = paired if isinstance(paired, list) else []
    episode_counts: dict[str, int] = {}
    missing_images = []
    for row in records:
        frame = row.get("frame") or row.get("frame_id") or row.get("sample_id") or ""
        if not frame:
            for value in row.values():
                if isinstance(value, str) and "episode_" in value:
                    frame = frame_from_source(value)
                    break
        episode = "_".join(frame.split("_")[:2]) if frame.startswith("episode_") else "UNKNOWN"
        episode_counts[episode] = episode_counts.get(episode, 0) + 1
        for key, value in row.items():
            if isinstance(value, str) and (value.endswith(".jpg") or value.endswith(".png")):
                if not Path(value).is_file():
                    missing_images.append(value)
    return {
        "manifest": str(paired_path),
        "record_count": len(records),
        "episode_counts": episode_counts,
        "missing_image_count": len(missing_images),
        "missing_images_first_20": missing_images[:20],
        "status": "PASS" if len(records) == 225 and not missing_images else "CHECK",
    }


def runtime_config_audit() -> dict[str, Any]:
    text = ROS_RUNTIME.read_text(encoding="utf-8") if ROS_RUNTIME.is_file() else ""
    checks = {
        "ros_runtime_exists": ROS_RUNTIME.is_file(),
        "bundle_checkpoint_exists": BUNDLE_CHECKPOINT.is_dir(),
        "ros_uses_proprio_variant": "oftplus_h5_proprio" in text,
        "ros_model_path": "/home/ubuntu/robot_ws/src/openvla/runs/oftplus_h5_proprio_bounded_oft200_9000--6000_chkpt"
        if "oftplus_h5_proprio_bounded_oft200_9000--6000_chkpt" in text
        else None,
        "bundle_checkpoint": str(BUNDLE_CHECKPOINT),
        "ros_center_crop_false": "center_crop_enabled: false" in text,
        "ros_use_training_preprocess_false": "use_training_image_preprocess: false" in text,
        "ros_default_instruction_pick_up_cube": 'default_instruction: "pick up the cube"' in text,
        "bundle_analysis_instruction": "Pick up the orange cube.",
        "bundle_analysis_variant": "oftplus_h5_vision",
    }
    mismatch_items = [
        "policy variant/checkpoint differs from bundle offline analysis",
        "ROS runtime uses proprio while bundle analysis used vision-only feature manifests",
        "ROS default instruction differs from bundle analysis instruction",
        "ROS preprocessing flags differ from bundle offline OFT feature extraction path",
    ]
    return {
        "ros_runtime_config": str(ROS_RUNTIME),
        "checks": checks,
        "mismatch_items": mismatch_items,
        "deployment_equivalence_status": "INVALID / RE-RUN REQUIRED",
    }


def summarize_existing_key_results() -> dict[str, Any]:
    obs = read_json(OUT / "rerun_observation_token_gap" / "observation_token_gap_summary.json")
    loo = read_json(OUT / "rerun_loo_real_to_sim_progress_shift" / "loo_summary.json")
    loo_features = loo["features"]
    policy = read_json(OUT / "rerun_policy_relevant_progress_shift" / "policy_relevant_progress_shift_summary.json")
    policy_metrics = policy["metrics"]
    collapse = read_json(LHJ / "phase2_validation" / "hidden_shift_no_collapse" / "hidden_shift_no_collapse_summary.json")
    layerwise = read_json(LHJ / "phase1_action_gap" / "layerwise_policy_relevance_summary.json")
    return {
        "observation": {
            "brightness_abs_diff_mean": obs["observation_gap"]["brightness_abs_diff"]["mean"],
            "rgb_mean_l2_mean": obs["observation_gap"]["rgb_mean_l2"]["mean"],
            "resized_psnr_mean": obs["observation_gap"]["resized_psnr"]["mean"],
            "resized_luma_ssim_global_mean": obs["observation_gap"]["resized_luma_ssim_global"]["mean"],
        },
        "loo_representation_shift": {
            "vision_cosine": [
                loo_features["vision_backbone.output"]["before_frame_cosine"]["mean"],
                loo_features["vision_backbone.output"]["after_frame_cosine"]["mean"],
            ],
            "projector_cosine": [
                loo_features["projector.output"]["before_frame_cosine"]["mean"],
                loo_features["projector.output"]["after_frame_cosine"]["mean"],
            ],
            "vision_mmd": [
                loo_features["vision_backbone.output"]["before_mmd"],
                loo_features["vision_backbone.output"]["after_mmd"],
            ],
            "projector_mmd": [
                loo_features["projector.output"]["before_mmd"],
                loo_features["projector.output"]["after_mmd"],
            ],
        },
        "policy_relevant_shift": {
            "hidden_shift_chunk_mean_l2": [
                policy_metrics["raw_chunk_mean_l2"]["mean"],
                policy_metrics["hidden_shift_chunk_mean_l2"]["mean"],
            ],
            "hidden_shift_first_l2": [
                policy_metrics["raw_first_l2"]["mean"],
                policy_metrics["hidden_shift_first_l2"]["mean"],
            ],
        },
        "no_collapse": {
            "variance_mean": {
                "real": collapse["variance"]["real"]["feature_variance_mean"],
                "sim": collapse["variance"]["sim"]["feature_variance_mean"],
                "corrected": collapse["variance"]["corrected"]["feature_variance_mean"],
            },
            "action_worse_frames": collapse["action_gap"]["worse_frame_count"],
            "hidden_l2_worse_frames": collapse["paired_gap"]["worse_l2_frame_count"],
        },
        "top_policy_relevance": layerwise["top_overall"][:5],
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    rerun_comparisons = [
        compare_json_numbers(
            LHJ / "phase1_observation_token_gap" / "observation_token_gap_summary.json",
            OUT / "rerun_observation_token_gap" / "observation_token_gap_summary.json",
        ),
        compare_json_numbers(
            LHJ / "phase1_audit" / "loo_real_to_sim_progress_shift" / "loo_summary.json",
            OUT / "rerun_loo_real_to_sim_progress_shift" / "loo_summary.json",
        ),
        compare_json_numbers(
            LHJ / "phase1_action_gap" / "policy_relevant_progress_shift" / "policy_relevant_progress_shift_summary.json",
            OUT / "rerun_policy_relevant_progress_shift" / "policy_relevant_progress_shift_summary.json",
        ),
    ]
    # repr-action summary contains generated path strings, but numeric values should match.
    rerun_comparisons.append(
        compare_json_numbers(
            LHJ / "phase1_action_gap" / "repr_action_gap_summary.json",
            OUT / "rerun_repr_action_gap" / "repr_action_gap_summary.json",
        )
    )

    item_rows = []
    summary = {
        "method": "comprehensive_offline_validation",
        "scope": "existing 5-episode offline bundle dataset only",
        "analysis_root": str(ANALYSIS_ROOT),
        "manifest_integrity": {
            "real": manifest_integrity("real"),
            "sim": manifest_integrity("sim"),
        },
        "paired_integrity": paired_integrity(),
        "rerun_comparisons": rerun_comparisons,
        "runtime_config_audit": runtime_config_audit(),
        "key_results": summarize_existing_key_results(),
        "limitations": [
            "No real robot motion was executed.",
            "Current ROS deployment policy/preprocessing is not equivalent to the bundle offline analysis.",
            "Camera/geometry/lighting causal attribution is not isolated by current paired data alone.",
            "Real-world performance remains unverified.",
        ],
    }

    statuses = {
        "real_manifest_integrity": summary["manifest_integrity"]["real"]["status"],
        "sim_manifest_integrity": summary["manifest_integrity"]["sim"]["status"],
        "paired_manifest_integrity": summary["paired_integrity"]["status"],
        "rerun_observation_token_gap": rerun_comparisons[0]["status"],
        "rerun_loo_representation_shift": rerun_comparisons[1]["status"],
        "rerun_policy_action_shift": rerun_comparisons[2]["status"],
        "rerun_repr_action_gap": rerun_comparisons[3]["status"],
        "deployment_equivalence": summary["runtime_config_audit"]["deployment_equivalence_status"],
        "real_robot_performance": "UNVERIFIED / REQUIRES REAL ROBOT APPROVAL",
        "causal_gap_attribution": "UNVERIFIED / REQUIRES CONTROLLED ABLATION",
    }
    summary["status_table"] = statuses
    for item, status in statuses.items():
        item_rows.append({"item": item, "status": status})
    write_json(OUT / "comprehensive_offline_validation_summary.json", summary)
    write_csv(OUT / "comprehensive_offline_validation_status.csv", item_rows)
    print(json.dumps(summary["status_table"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
