from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def validate(episode_dir: Path) -> None:
    metadata_path = episode_dir / "metadata.json"
    steps_path = episode_dir / "steps_with_actions.jsonl"

    if not metadata_path.exists():
        raise FileNotFoundError(metadata_path)
    if not steps_path.exists():
        raise FileNotFoundError(steps_path)

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    steps = [
        json.loads(line)
        for line in steps_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]

    errors: list[str] = []
    previous_timestamp = -1.0

    for i, step in enumerate(steps):
        pose = np.asarray(step.get("tcp_pose"), dtype=np.float64)
        action = np.asarray(step.get("action"), dtype=np.float64)
        timestamp = float(step.get("timestamp", -1.0))
        gripper = step.get("gripper")

        if pose.shape != (6,) or not np.isfinite(pose).all():
            errors.append(f"step {i}: invalid tcp_pose")
        if action.shape != (7,) or not np.isfinite(action).all():
            errors.append(f"step {i}: invalid action")
        if gripper not in (0, 1):
            errors.append(f"step {i}: gripper must be 0 or 1")
        if timestamp < previous_timestamp:
            errors.append(f"step {i}: timestamp decreased")
        previous_timestamp = timestamp

        image_path = episode_dir / step.get("image", "")
        image = cv2.imread(str(image_path))
        if image is None:
            errors.append(f"step {i}: invalid image {image_path}")

    if metadata.get("num_steps") != len(steps):
        errors.append(
            f"metadata num_steps={metadata.get('num_steps')} "
            f"but file has {len(steps)}"
        )

    if errors:
        print("Validation failed:")
        for error in errors:
            print(f"  - {error}")
        raise SystemExit(1)

    print(
        f"Validation passed: {episode_dir} "
        f"({len(steps)} steps, success={metadata.get('success')})"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode_dir", type=Path)
    args = parser.parse_args()
    validate(args.episode_dir)


if __name__ == "__main__":
    main()
