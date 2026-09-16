#!/usr/bin/env python3
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import torch


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
LHJ = ROOT / "lhj"
OUT = LHJ / "phase3_oft_variant_audit"
VISION_CKPT = ROOT / "runtime_state" / "oft_mixed480_step28560_merged"
PROPRIO_CKPT = Path("/home/ubuntu/robot_ws/src/openvla/runs/oftplus_h5_proprio_bounded_oft200_9000--6000_chkpt")
ROS_RUNTIME = Path("/home/ubuntu/robot_ws/src/openvla_doosan_runtime/config/runtime_oft.yaml")
ANALYSIS_ROOT = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes"


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


def sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_pt_shapes(path: Path) -> dict[str, list[int]]:
    if not path.is_file():
        return {}
    state = torch.load(path, map_location="cpu")
    if isinstance(state, dict) and "state_dict" in state and isinstance(state["state_dict"], dict):
        state = state["state_dict"]
    out = {}
    for key, value in state.items():
        clean_key = key[7:] if key.startswith("module.") else key
        if hasattr(value, "shape"):
            out[clean_key] = list(value.shape)
    return out


def first_matching_file(root: Path, pattern: str) -> Path | None:
    matches = sorted(root.glob(pattern))
    return matches[0] if matches else None


def component_summary(root: Path) -> dict[str, Any]:
    files = {
        "action_head": first_matching_file(root, "action_head*checkpoint.pt"),
        "vision_backbone": first_matching_file(root, "vision_backbone*checkpoint.pt"),
        "proprio_projector": first_matching_file(root, "proprio_projector*checkpoint.pt"),
        "noisy_action_projector": first_matching_file(root, "noisy_action_projector*checkpoint.pt"),
    }
    output = {}
    for name, path in files.items():
        shapes = load_pt_shapes(path) if path is not None else {}
        output[name] = {
            "path": str(path) if path is not None else None,
            "exists": path is not None and path.is_file(),
            "sha256": sha256(path) if path is not None else None,
            "param_count": len(shapes),
            "shapes": shapes,
        }
    return output


def shape_compare(left: dict[str, list[int]], right: dict[str, list[int]]) -> dict[str, Any]:
    left_keys = set(left)
    right_keys = set(right)
    common = sorted(left_keys & right_keys)
    shape_mismatch = [
        {"name": key, "left": left[key], "right": right[key]}
        for key in common
        if left[key] != right[key]
    ]
    return {
        "left_only": sorted(left_keys - right_keys),
        "right_only": sorted(right_keys - left_keys),
        "common_count": len(common),
        "shape_mismatch_count": len(shape_mismatch),
        "shape_mismatch_first_20": shape_mismatch[:20],
        "same_shape_for_common": not shape_mismatch,
    }


def model_index_summary(root: Path) -> dict[str, Any]:
    index_path = root / "model.safetensors.index.json"
    if not index_path.is_file():
        return {"path": str(index_path), "exists": False}
    index = read_json(index_path)
    weight_map = index.get("weight_map", {})
    shard_counts: dict[str, int] = {}
    for shard in weight_map.values():
        shard_counts[shard] = shard_counts.get(shard, 0) + 1
    return {
        "path": str(index_path),
        "exists": True,
        "metadata": index.get("metadata", {}),
        "weight_count": len(weight_map),
        "shard_counts": shard_counts,
        "first_20_weight_names": sorted(weight_map)[:20],
    }


def feature_manifest_summary(domain: str) -> dict[str, Any]:
    manifest_path = ANALYSIS_ROOT / "features" / domain / "feature_manifest.json"
    manifest = read_json(manifest_path)
    records = manifest["records"]
    variants = sorted({record.get("response", {}).get("variant") for record in records})
    instructions = sorted({record.get("instruction") for record in records})
    normalized_instructions = sorted({record.get("response", {}).get("normalized_instruction") for record in records})
    sample = records[0]
    feature_keys = sorted(sample.get("features", {}).keys())
    feature_shapes = {
        key: sample["features"][key].get("shape")
        for key in feature_keys
        if isinstance(sample.get("features", {}).get(key), dict)
    }
    return {
        "path": str(manifest_path),
        "record_count": len(records),
        "manifest_checkpoint": manifest.get("checkpoint"),
        "model": manifest.get("model"),
        "domain": manifest.get("domain"),
        "full_features": manifest.get("full_features"),
        "variants": variants,
        "instructions": instructions,
        "normalized_instructions": normalized_instructions,
        "first_feature_file": sample.get("feature_file"),
        "feature_keys": feature_keys,
        "feature_shapes_first_record": feature_shapes,
    }


def dataset_stats_summary(root: Path) -> dict[str, Any]:
    path = root / "dataset_statistics.json"
    if not path.is_file():
        return {"path": str(path), "exists": False}
    stats = read_json(path)
    key = "a0509_sim_cube_pick"
    entry = stats.get(key, {})
    action = entry.get("action", {})
    proprio = entry.get("proprio", {})
    return {
        "path": str(path),
        "exists": True,
        "keys": sorted(stats.keys()),
        "action_keys": sorted(action.keys()),
        "action_q01": action.get("q01"),
        "action_q99": action.get("q99"),
        "action_mask": action.get("mask"),
        "has_proprio_stats": bool(proprio),
        "proprio_keys": sorted(proprio.keys()) if isinstance(proprio, dict) else [],
        "proprio_q01": proprio.get("q01") if isinstance(proprio, dict) else None,
        "proprio_q99": proprio.get("q99") if isinstance(proprio, dict) else None,
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    vision_config = read_json(VISION_CKPT / "a0509_training_config.json")
    proprio_config = read_json(PROPRIO_CKPT / "a0509_training_config.json")
    vision_components = component_summary(VISION_CKPT)
    proprio_components = component_summary(PROPRIO_CKPT)
    component_shape_comparison = {}
    for component in sorted(set(vision_components) | set(proprio_components)):
        component_shape_comparison[component] = shape_compare(
            vision_components.get(component, {}).get("shapes", {}),
            proprio_components.get(component, {}).get("shapes", {}),
        )

    rows = []
    for key in sorted(set(vision_config) | set(proprio_config)):
        rows.append(
            {
                "config_key": key,
                "vision": json.dumps(vision_config.get(key), ensure_ascii=False),
                "proprio": json.dumps(proprio_config.get(key), ensure_ascii=False),
                "same": vision_config.get(key) == proprio_config.get(key),
            }
        )
    write_csv(OUT / "training_config_compare.csv", rows)

    shape_rows = []
    for component, comparison in component_shape_comparison.items():
        shape_rows.append(
            {
                "component": component,
                "vision_exists": vision_components.get(component, {}).get("exists"),
                "proprio_exists": proprio_components.get(component, {}).get("exists"),
                "common_count": comparison["common_count"],
                "left_only_count": len(comparison["left_only"]),
                "right_only_count": len(comparison["right_only"]),
                "shape_mismatch_count": comparison["shape_mismatch_count"],
                "same_shape_for_common": comparison["same_shape_for_common"],
            }
        )
    write_csv(OUT / "component_shape_compare.csv", shape_rows)

    summary = {
        "scope": "oftplus_h5_vision vs oftplus_h5_proprio audit",
        "vision_checkpoint": str(VISION_CKPT),
        "proprio_checkpoint": str(PROPRIO_CKPT),
        "ros_runtime_config": str(ROS_RUNTIME),
        "architecture_judgement": {
            "category": "C. same base OpenVLA/OFT model class with proprio branch adding an extra learned proprio token when enabled",
            "reason": [
                "Variants are registered by booleans use_film/use_proprio/use_l1_regression/use_diffusion.",
                "Both target variants use L1 regression and FiLM.",
                "Proprio variant additionally requires/loads ProprioProjector and passes normalized 7D state.",
                "Modeling code appends one proprio token to projected vision patch embeddings before multimodal/action prediction.",
                "Action head class is same L1RegressionActionHead shape for both checkpoints.",
            ],
        },
        "vision_training_config": vision_config,
        "proprio_training_config": proprio_config,
        "component_summary": {
            "vision": vision_components,
            "proprio": proprio_components,
        },
        "component_shape_comparison": component_shape_comparison,
        "model_index_summary": {
            "vision": model_index_summary(VISION_CKPT),
            "proprio": model_index_summary(PROPRIO_CKPT),
        },
        "dataset_statistics": {
            "vision": dataset_stats_summary(VISION_CKPT),
            "proprio": dataset_stats_summary(PROPRIO_CKPT),
        },
        "feature_manifest_provenance": {
            "real": feature_manifest_summary("real"),
            "sim": feature_manifest_summary("sim"),
            "extraction_script": str(ROOT / "sim2real_analysis" / "04_features" / "extract_vla_features.py"),
            "runtime_script": str(ROOT / "runtime" / "openvla-oft" / "vla-scripts" / "serve_a0509_oft.py"),
        },
        "equivalence_judgement": {
            "architecture_equivalence": "PARTIAL: same model class and same L1/FiLM/action-head shape, but proprio branch adds ProprioProjector and one extra token in the proprio path.",
            "checkpoint_equivalence": "NO: distinct checkpoints, distinct step numbers, distinct action/vision component files; proprio checkpoint has additional proprio_projector checkpoint.",
            "input_interface_equivalence": "NO: vision variant consumes image only; proprio variant consumes image plus normalized 7D proprio state.",
            "preprocessing_equivalence": "NO for current ROS deployment: offline extraction uses center_crop=True in OFTRuntime; runtime_oft.yaml sets use_training_image_preprocess=false and center_crop_enabled=false.",
            "action_interface_equivalence": "PARTIAL: both output 5x7 chunks and use a0509_sim_cube_pick action stats, but checkpoint/action_head weights and runtime downstream action adapter differ.",
        },
    }
    write_json(OUT / "oft_vision_proprio_audit_summary.json", summary)
    print(json.dumps(summary["equivalence_judgement"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
