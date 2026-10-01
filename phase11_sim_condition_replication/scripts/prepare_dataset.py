#!/usr/bin/env python3
"""Prepare JSON manifests/layouts/USD overlays for the 5x4 paired Sim dataset.

This script does not launch Isaac Sim. Baseline episode data are referenced in
place; they are never copied or modified.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "experiment.json"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def lighting_overlay(scene: Path, fill: float, key: float) -> str:
    return f'''#usda 1.0
(
    subLayers = [@{scene}@]
)

over "World" {{
    over "Environment" {{
        over "Lighting" {{
            over "FillLight" {{
                float inputs:intensity = {fill}
            }}
            over "KeyLight" {{
                float inputs:intensity = {key}
            }}
        }}
    }}
}}
'''


def object_overlay(scene: Path, obj: dict) -> str:
    x, y, z = obj["position_m"]
    r, h = obj["radius_m"], obj["height_m"]
    c = ", ".join(str(v) for v in obj["display_color"])
    return f'''#usda 1.0
(
    subLayers = [@{scene}@]
)

over "World" {{
    over "Environment" {{
        over "Objects" {{
            def Cylinder "Phase11ExtraObject" (
                displayName = "Phase 11 additional object"
            ) {{
                float radius = {r}
                float height = {h}
                color3f[] primvars:displayColor = [({c})]
                double3 xformOp:translate = ({x}, {y}, {z})
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }}
        }}
    }}
}}
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    cfg = load_json(args.config.resolve())
    dataset = Path(cfg["dataset_root"])
    generated = dataset / "generated"
    generated.mkdir(parents=True, exist_ok=True)
    scene = Path(cfg["scene"]).resolve()

    light_scene = generated / "scene_lighting_low.usda"
    light_scene.write_text(
        lighting_overlay(
            scene,
            cfg["lighting_low"]["fill_intensity"],
            cfg["lighting_low"]["key_intensity"],
        ), encoding="utf-8"
    )
    object_scene = generated / "scene_extra_object.usda"
    object_scene.write_text(object_overlay(scene, cfg["extra_object"]), encoding="utf-8")

    rows = []
    for episode_num in cfg["base_episode_ids"]:
        episode_id = f"episode_{episode_num:06d}"
        source_root = Path(cfg["source_sim_root"])
        source_real = Path(cfg["source_real_episode_root"]) / episode_id
        base_layout_path = source_root / f"trajectory{episode_num:03d}_manual_layout.json"
        base_layout = load_json(base_layout_path)
        swapped = json.loads(json.dumps(base_layout))
        swapped["layout_id"] = f"{base_layout.get('layout_id', episode_id)}_distractor_swap"
        swapped["condition_id"] = "distractor_swap"
        swapped["source_layout"] = str(base_layout_path)
        swapped["layout_m"]["yellow"], swapped["layout_m"]["blue"] = (
            swapped["layout_m"]["blue"], swapped["layout_m"]["yellow"]
        )
        if "real_layout_mm" in swapped:
            swapped["real_layout_mm"]["yellow"], swapped["real_layout_mm"]["blue"] = (
                swapped["real_layout_mm"]["blue"], swapped["real_layout_mm"]["yellow"]
            )
        swapped_path = generated / f"{episode_id}_distractor_swap_layout.json"
        swapped_path.write_text(json.dumps(swapped, indent=2), encoding="utf-8")

        source_baseline_report = source_root / f"raw_dataset_oft_{episode_id}_home_relative_joint_replay.json"
        source_baseline_episode = source_root / f"raw_dataset_oft_{episode_id}_home_relative_joint_replay_images"
        baseline_root = dataset / "episodes" / "baseline"
        baseline_root.mkdir(parents=True, exist_ok=True)
        baseline_report = baseline_root / f"baseline_{episode_id}.json"
        baseline_episode = baseline_root / f"baseline_{episode_id}_images"
        if cfg["storage"].get("baseline_copy", False):
            if not baseline_report.exists():
                shutil.copy2(source_baseline_report, baseline_report)
            if not baseline_episode.exists():
                shutil.copytree(source_baseline_episode, baseline_episode)
        common = {
            "base_episode_id": episode_id,
            "source_steps_jsonl": str(source_real / "steps.jsonl"),
            "source_layout_json": str(base_layout_path),
            "source_layout_sha256": sha256(base_layout_path),
            "instruction": cfg["instruction"],
            "trajectory_invariant": True,
            "target_orange_position_invariant": True,
        }
        rows.append({
            **common, "condition_id": "baseline", "collection_status": "EXISTING_REFERENCE",
            "scene": str(scene), "layout_json": str(base_layout_path),
            "report_json": str(baseline_report), "episode_dir": str(baseline_episode),
            "source_report_json": str(source_baseline_report),
            "source_episode_dir": str(source_baseline_episode),
        })
        for condition, condition_scene, layout in (
            ("lighting_low", light_scene, base_layout_path),
            ("extra_object", object_scene, base_layout_path),
            ("distractor_swap", scene, swapped_path),
        ):
            stem = f"{condition}_{episode_id}"
            rows.append({
                **common, "condition_id": condition, "collection_status": "PENDING",
                "scene": str(condition_scene), "layout_json": str(layout),
                "report_json": str(dataset / "episodes" / condition / f"{stem}.json"),
                "episode_dir": str(dataset / "episodes" / condition / f"{stem}_images"),
            })

    manifest = {
        "schema_version": 1,
        "experiment_id": cfg["experiment_id"],
        "design": "5 base trajectories x 4 paired conditions",
        "episode_count_total": len(rows),
        "existing_baseline_count": sum(r["condition_id"] == "baseline" for r in rows),
        "new_collection_count": sum(r["condition_id"] != "baseline" for r in rows),
        "unit_of_independence": "base trajectory/layout, not frame",
        "target_cube_policy": "orange/red target position fixed within each paired group",
        "records": rows,
    }
    manifest_path = dataset / "dataset_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"manifest": str(manifest_path), "total": len(rows), "new": 15}, indent=2))


if __name__ == "__main__":
    main()
