from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .rlds_utils import (
    EpisodeValidationError,
    discover_recorded_episodes,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate raw episodes before TFDS/RLDS conversion."
    )
    parser.add_argument(
        "raw_dataset_root",
        nargs="?",
        type=Path,
        default=Path("raw_dataset"),
        help="Directory containing episode_* folders.",
    )
    parser.add_argument(
        "--include-failures",
        action="store_true",
        help="Include episodes whose metadata success flag is false.",
    )
    args = parser.parse_args()

    try:
        episodes = discover_recorded_episodes(
            args.raw_dataset_root,
            success_only=not args.include_failures,
            require_actions=True,
        )
    except EpisodeValidationError as exc:
        raise SystemExit(f"RLDS source validation failed: {exc}") from exc

    total_steps = sum(len(episode.steps) for episode in episodes)
    first = episodes[0].steps[0]
    first_pose = np.asarray(first["tcp_pose"], dtype=np.float32)
    first_action = np.asarray(first["action"], dtype=np.float32)

    print(f"Convertible episodes : {len(episodes)}")
    print(f"Total steps          : {total_steps}")
    print(f"First episode        : {episodes[0].episode_id}")
    print(f"Instruction          : {episodes[0].instruction}")
    print("Dataset units        : position=meter, rotation=radian")
    print("Action shape         : (7,)")
    print(f"First tcp_pose       : {first_pose.tolist()}")
    print(f"First action         : {first_action.tolist()}")


if __name__ == "__main__":
    main()
