from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Iterator

import numpy as np
import tensorflow_datasets as tfds

try:
    from vla_data_collector.rlds_utils import (
        RecordedEpisode,
        discover_recorded_episodes,
        read_rgb_image,
        split_episodes,
        zero_language_embedding,
    )
except ModuleNotFoundError:
    import sys

    repository_root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(repository_root / "vla_data_collector"))
    from vla_data_collector.rlds_utils import (
        RecordedEpisode,
        discover_recorded_episodes,
        read_rgb_image,
        split_episodes,
        zero_language_embedding,
    )


class DoosanA0509(tfds.core.GeneratorBasedBuilder):
    """RLDS builder for Doosan A0509 VLA episodes."""

    VERSION = tfds.core.Version("1.0.0")
    RELEASE_NOTES = {
        "1.0.0": "Initial RLDS conversion for Doosan A0509 VLA episodes.",
    }

    def _info(self) -> tfds.core.DatasetInfo:
        return self.dataset_info_from_configs(
            features=tfds.features.FeaturesDict(
                {
                    "steps": tfds.features.Dataset(
                        {
                            "observation": tfds.features.FeaturesDict(
                                {
                                    "image": tfds.features.Image(
                                        shape=(None, None, 3),
                                        dtype=np.uint8,
                                        encoding_format="jpeg",
                                        doc="Eye-in-hand ZED RGB image.",
                                    ),
                                    "EEF_state": tfds.features.Tensor(
                                        shape=(6,),
                                        dtype=np.float32,
                                        doc=(
                                            "End-effector pose "
                                            "[x, y, z, roll, pitch, yaw] "
                                            "in meters and radians."
                                        ),
                                    ),
                                    "gripper_state": tfds.features.Tensor(
                                        shape=(1,),
                                        dtype=np.float32,
                                        doc="Gripper state, 0=closed and 1=open.",
                                    ),
                                    "state": tfds.features.Tensor(
                                        shape=(7,),
                                        dtype=np.float32,
                                        doc=(
                                            "EEF_state concatenated with "
                                            "gripper_state."
                                        ),
                                    ),
                                }
                            ),
                            "action": tfds.features.Tensor(
                                shape=(7,),
                                dtype=np.float32,
                                doc=(
                                    "Delta action "
                                    "[dx, dy, dz, droll, dpitch, dyaw, "
                                    "gripper] in meters/radians plus "
                                    "absolute gripper target."
                                ),
                            ),
                            "discount": tfds.features.Scalar(
                                dtype=np.float32,
                                doc="Discount, fixed to 1.0 for demos.",
                            ),
                            "reward": tfds.features.Scalar(
                                dtype=np.float32,
                                doc="1.0 on final successful step, else 0.0.",
                            ),
                            "is_first": tfds.features.Scalar(
                                dtype=np.bool_,
                                doc="True on the first step.",
                            ),
                            "is_last": tfds.features.Scalar(
                                dtype=np.bool_,
                                doc="True on the final step.",
                            ),
                            "is_terminal": tfds.features.Scalar(
                                dtype=np.bool_,
                                doc="True on the final step.",
                            ),
                            "language_instruction": tfds.features.Text(
                                doc="Natural language task instruction.",
                            ),
                            "language_embedding": tfds.features.Tensor(
                                shape=(512,),
                                dtype=np.float32,
                                doc=(
                                    "Language embedding placeholder. "
                                    "OpenVLA uses language_instruction."
                                ),
                            ),
                        }
                    ),
                    "episode_metadata": tfds.features.FeaturesDict(
                        {
                            "file_path": tfds.features.Text(
                                doc="Original raw episode directory."
                            ),
                            "episode_id": tfds.features.Text(
                                doc="Episode identifier."
                            ),
                            "success": tfds.features.Scalar(
                                dtype=np.bool_,
                                doc="Episode success flag from metadata.",
                            ),
                            "num_steps": tfds.features.Scalar(
                                dtype=np.int32,
                                doc="Number of timesteps in this episode.",
                            ),
                        }
                    ),
                }
            )
        )

    def _split_generators(self, dl_manager: tfds.download.DownloadManager):
        raw_root = _resolve_raw_dataset_root(dl_manager)
        val_ratio = _read_float_env("DOOSAN_A0509_VAL_RATIO", default=0.0)
        success_only = not _read_bool_env(
            "DOOSAN_A0509_INCLUDE_FAILURES",
            default=False,
        )

        episodes = discover_recorded_episodes(
            raw_root,
            success_only=success_only,
            require_actions=True,
        )
        train_episodes, val_episodes = split_episodes(
            episodes,
            val_ratio=val_ratio,
        )

        splits = {
            "train": self._generate_examples(train_episodes),
        }
        if val_episodes:
            splits["val"] = self._generate_examples(val_episodes)
        return splits

    def _generate_examples(
        self,
        episodes: list[RecordedEpisode],
    ) -> Iterator[tuple[str, dict[str, Any]]]:
        for episode in episodes:
            yield episode.episode_id, _convert_episode(episode)


def _convert_episode(episode: RecordedEpisode) -> dict[str, Any]:
    step_count = len(episode.steps)
    instruction = episode.instruction
    language_embedding = zero_language_embedding()
    converted_steps = []

    for index, step in enumerate(episode.steps):
        tcp_pose = np.asarray(step["tcp_pose"], dtype=np.float32)
        gripper = np.asarray([float(step["gripper"])], dtype=np.float32)
        action = np.asarray(step["action"], dtype=np.float32)
        image = read_rgb_image(episode.path / step["image"])
        is_last = index == step_count - 1

        converted_steps.append(
            {
                "observation": {
                    "image": image,
                    "EEF_state": tcp_pose,
                    "gripper_state": gripper,
                    "state": np.concatenate([tcp_pose, gripper]).astype(
                        np.float32
                    ),
                },
                "action": action,
                "discount": np.float32(1.0),
                "reward": np.float32(float(is_last and episode.success)),
                "is_first": index == 0,
                "is_last": is_last,
                "is_terminal": is_last,
                "language_instruction": instruction,
                "language_embedding": language_embedding.copy(),
            }
        )

    return {
        "steps": converted_steps,
        "episode_metadata": {
            "file_path": str(episode.path),
            "episode_id": episode.episode_id,
            "success": episode.success,
            "num_steps": np.int32(step_count),
        },
    }


def _resolve_raw_dataset_root(
    dl_manager: tfds.download.DownloadManager,
) -> Path:
    env_path = os.environ.get("DOOSAN_A0509_RAW_DATASET_DIR")
    if env_path:
        return Path(env_path).expanduser().resolve()

    manual_dir = Path(dl_manager.manual_dir).expanduser()
    if manual_dir.exists() and any(manual_dir.glob("episode_*")):
        return manual_dir.resolve()

    cwd = Path.cwd().resolve()
    candidates = [cwd / "raw_dataset"]
    candidates.extend(parent / "raw_dataset" for parent in cwd.parents)

    for candidate in candidates:
        if candidate.exists() and any(candidate.glob("episode_*")):
            return candidate.resolve()

    raise FileNotFoundError(
        "Could not locate raw_dataset. Set DOOSAN_A0509_RAW_DATASET_DIR "
        "or run tfds build with --manual_dir pointing to raw_dataset."
    )


def _read_bool_env(name: str, *, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default

    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def _read_float_env(name: str, *, default: float) -> float:
    value = os.environ.get(name)
    if value is None:
        return default

    try:
        return float(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be a float, got {value!r}") from exc
