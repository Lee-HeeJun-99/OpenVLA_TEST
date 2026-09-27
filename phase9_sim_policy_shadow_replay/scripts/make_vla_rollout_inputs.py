#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prepare a fixed-layout manifest for Isaac Sim VLA policy rollout."
    )
    parser.add_argument("--manual-layout-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--target-color", default="orange")
    parser.add_argument("--instruction", default="Pick up the orange cube.")
    return parser.parse_args()


def _jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def main() -> None:
    args = parse_args()
    if args.episodes < 1:
        raise ValueError("--episodes must be positive")

    layout_path = args.manual_layout_json.expanduser().resolve()
    payload = json.loads(layout_path.read_text(encoding="utf-8"))
    layout_m = payload.get("layout_m", payload.get("layout"))
    if not isinstance(layout_m, dict):
        raise ValueError(f"Layout has no layout_m/layout object: {layout_path}")
    expected = {"red", "yellow", "blue"}
    if set(layout_m) != expected:
        raise ValueError(f"Layout must define {sorted(expected)}, got {sorted(layout_m)}")
    for color, xyz in layout_m.items():
        if not isinstance(xyz, list) or len(xyz) != 3:
            raise ValueError(f"Invalid XYZ for {color}: {xyz}")
        [float(value) for value in xyz]

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    layout_snapshot = output_dir / "source_manual_layout.json"
    shutil.copy2(layout_path, layout_snapshot)

    manifest = {
        "description": (
            "Fixed layout manifest for Isaac Sim closed-loop VLA policy rollout. "
            "The manifest uses target_color=orange. Isaac Sim internally maps "
            "orange to the red prim in the current A0509 scene."
        ),
        "source_manual_layout_json": str(layout_path),
        "instruction": args.instruction,
        "episodes": [
            {
                "episode_index": index,
                "target_color": args.target_color,
                "instruction": args.instruction,
                "layout_m": layout_m,
            }
            for index in range(args.episodes)
        ],
    }
    manifest_path = output_dir / "vla_layout_manifest.json"
    manifest_path.write_text(
        json.dumps(_jsonable(manifest), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(
        json.dumps(
            {
                "manifest": str(manifest_path),
                "layout_snapshot": str(layout_snapshot),
                "episodes": args.episodes,
                "target_color": args.target_color,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
