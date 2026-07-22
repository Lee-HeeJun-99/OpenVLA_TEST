from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def wrap_to_pi(values: np.ndarray) -> np.ndarray:
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def calculate_action(current: dict, following: dict) -> list[float]:
    current_pose = np.asarray(current["tcp_pose"], dtype=np.float64)
    next_pose = np.asarray(following["tcp_pose"], dtype=np.float64)

    if current_pose.shape != (6,) or next_pose.shape != (6,):
        raise ValueError("TCP pose must have six values")
    if not np.isfinite(current_pose).all() or not np.isfinite(next_pose).all():
        raise ValueError("TCP pose contains NaN or infinity")

    delta_position = next_pose[:3] - current_pose[:3]
    delta_rotation = wrap_to_pi(next_pose[3:] - current_pose[3:])
    gripper = float(following["gripper"])

    return np.concatenate(
        [delta_position, delta_rotation, [gripper]]
    ).astype(np.float32).tolist()


def process_episode(episode_dir: Path) -> Path:
    input_path = episode_dir / "steps.jsonl"
    if not input_path.exists():
        raise FileNotFoundError(input_path)

    steps = [
        json.loads(line)
        for line in input_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if len(steps) < 2:
        raise ValueError("At least two steps are required")

    for i in range(len(steps) - 1):
        steps[i]["action"] = calculate_action(steps[i], steps[i + 1])

    steps[-1]["action"] = [
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        float(steps[-1]["gripper"]),
    ]

    output_path = episode_dir / "steps_with_actions.jsonl"
    with output_path.open("w", encoding="utf-8") as fp:
        for step in steps:
            fp.write(json.dumps(step, ensure_ascii=False) + "\n")

    return output_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode_dir", type=Path)
    args = parser.parse_args()

    output = process_episode(args.episode_dir)
    print(f"Saved: {output}")


if __name__ == "__main__":
    main()
