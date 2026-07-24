from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

repository_root = Path(__file__).resolve().parents[2]
sys.path.insert(
    0,
    str(repository_root / "vla_data_collector"),
)

from doosan_a0509_dataset_builder import DoosanA0509
from vla_data_collector.rlds_utils import discover_recorded_episodes


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build the Doosan A0509 TFDS/RLDS dataset."
    )
    parser.add_argument(
        "--raw-dataset-dir",
        type=Path,
        default=Path("~/robot_ws/raw_dataset"),
        help="Directory containing episode_* raw dataset folders.",
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path("~/tensorflow_datasets"),
        help="TFDS output root.",
    )
    parser.add_argument(
        "--val-ratio",
        type=float,
        default=0.0,
        help="Validation split ratio in [0.0, 1.0).",
    )
    parser.add_argument(
        "--include-failures",
        action="store_true",
        help="Include episodes with success=false.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove the existing doosan_a0509 version directory first.",
    )
    args = parser.parse_args()

    raw_dataset_dir = args.raw_dataset_dir.expanduser().resolve()
    data_dir = args.data_dir.expanduser().resolve()

    os.environ["DOOSAN_A0509_RAW_DATASET_DIR"] = str(raw_dataset_dir)
    os.environ["DOOSAN_A0509_VAL_RATIO"] = str(args.val_ratio)
    os.environ["DOOSAN_A0509_INCLUDE_FAILURES"] = (
        "1" if args.include_failures else "0"
    )

    episodes = discover_recorded_episodes(
        raw_dataset_dir,
        success_only=not args.include_failures,
        require_actions=True,
    )

    builder = DoosanA0509(data_dir=str(data_dir))
    output_dir = Path(str(builder.data_path))

    if output_dir.exists() and args.overwrite:
        shutil.rmtree(output_dir)

    print(f"Raw dataset : {raw_dataset_dir}")
    print(f"Episodes    : {len(episodes)}")
    print(f"TFDS output : {output_dir}")
    print(f"Overwrite   : {args.overwrite}")

    builder.download_and_prepare()

    print(f"Saved RLDS dataset: {output_dir}")


if __name__ == "__main__":
    main()
