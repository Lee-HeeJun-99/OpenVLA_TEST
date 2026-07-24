from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from PIL import Image


class EpisodeValidationError(ValueError):
    """Raised when a recorded episode cannot be converted to RLDS."""


@dataclass(frozen=True)
class RecordedEpisode:
    path: Path
    metadata: dict[str, Any]
    steps: list[dict[str, Any]]

    @property
    def episode_id(self) -> str:
        return str(self.metadata.get("episode_id", self.path.name))

    @property
    def instruction(self) -> str:
        instruction = str(self.metadata.get("instruction", "")).strip()
        return instruction or "pick up the cube"

    @property
    def success(self) -> bool:
        return bool(self.metadata.get("success", False))


def iter_episode_dirs(raw_dataset_root: Path) -> list[Path]:
    root = raw_dataset_root.expanduser().resolve()
    if not root.exists():
        raise FileNotFoundError(f"Raw dataset root not found: {root}")
    if not root.is_dir():
        raise NotADirectoryError(f"Raw dataset root is not a directory: {root}")

    return sorted(
        path
        for path in root.iterdir()
        if path.is_dir() and path.name.startswith("episode_")
    )


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise EpisodeValidationError(f"Invalid JSON: {path}") from exc

    if not isinstance(value, dict):
        raise EpisodeValidationError(f"JSON root must be an object: {path}")

    return value


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise EpisodeValidationError(f"Cannot read JSONL: {path}") from exc

    for line_index, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise EpisodeValidationError(
                f"Invalid JSONL at {path}:{line_index}"
            ) from exc

        if not isinstance(value, dict):
            raise EpisodeValidationError(
                f"JSONL record must be an object at {path}:{line_index}"
            )
        records.append(value)

    return records


def _as_finite_vector(
    value: Any,
    *,
    length: int,
    name: str,
    episode_dir: Path,
    step_index: int,
) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError) as exc:
        raise EpisodeValidationError(
            f"{episode_dir.name} step {step_index}: {name} is not numeric"
        ) from exc

    if array.shape != (length,):
        raise EpisodeValidationError(
            f"{episode_dir.name} step {step_index}: "
            f"{name} must have shape ({length},), got {array.shape}"
        )
    if not np.isfinite(array).all():
        raise EpisodeValidationError(
            f"{episode_dir.name} step {step_index}: {name} contains NaN/Inf"
        )

    return array


def _validate_gripper(
    value: Any,
    *,
    episode_dir: Path,
    step_index: int,
) -> int:
    try:
        gripper = int(value)
    except (TypeError, ValueError) as exc:
        raise EpisodeValidationError(
            f"{episode_dir.name} step {step_index}: gripper is not an integer"
        ) from exc

    if gripper not in (0, 1):
        raise EpisodeValidationError(
            f"{episode_dir.name} step {step_index}: gripper must be 0 or 1"
        )

    return gripper


def validate_step(
    episode_dir: Path,
    step: dict[str, Any],
    *,
    expected_index: int,
    require_action: bool,
) -> None:
    step_index = int(step.get("step_index", expected_index))
    if step_index != expected_index:
        raise EpisodeValidationError(
            f"{episode_dir.name}: expected step_index {expected_index}, "
            f"got {step_index}"
        )

    image_value = step.get("image")
    if not isinstance(image_value, str) or not image_value:
        raise EpisodeValidationError(
            f"{episode_dir.name} step {step_index}: image path is missing"
        )

    image_path = episode_dir / image_value
    if not image_path.exists():
        raise EpisodeValidationError(
            f"{episode_dir.name} step {step_index}: image not found: "
            f"{image_path}"
        )

    _as_finite_vector(
        step.get("tcp_pose"),
        length=6,
        name="tcp_pose",
        episode_dir=episode_dir,
        step_index=step_index,
    )
    _validate_gripper(
        step.get("gripper"),
        episode_dir=episode_dir,
        step_index=step_index,
    )

    if require_action:
        _as_finite_vector(
            step.get("action"),
            length=7,
            name="action",
            episode_dir=episode_dir,
            step_index=step_index,
        )


def load_recorded_episode(
    episode_dir: Path,
    *,
    require_actions: bool = True,
) -> RecordedEpisode:
    episode_dir = episode_dir.expanduser().resolve()
    metadata_path = episode_dir / "metadata.json"
    steps_path = (
        episode_dir / "steps_with_actions.jsonl"
        if require_actions
        else episode_dir / "steps.jsonl"
    )

    if not metadata_path.exists():
        raise EpisodeValidationError(f"Missing metadata: {metadata_path}")
    if not steps_path.exists():
        raise EpisodeValidationError(f"Missing steps file: {steps_path}")

    metadata = read_json(metadata_path)
    steps = read_jsonl(steps_path)
    if not steps:
        raise EpisodeValidationError(f"No steps found: {steps_path}")

    for index, step in enumerate(steps):
        validate_step(
            episode_dir,
            step,
            expected_index=index,
            require_action=require_actions,
        )

    metadata_steps = metadata.get("num_steps")
    if metadata_steps is not None and int(metadata_steps) != len(steps):
        raise EpisodeValidationError(
            f"{episode_dir.name}: metadata num_steps={metadata_steps}, "
            f"but steps file has {len(steps)}"
        )

    return RecordedEpisode(
        path=episode_dir,
        metadata=metadata,
        steps=steps,
    )


def discover_recorded_episodes(
    raw_dataset_root: Path,
    *,
    success_only: bool = True,
    require_actions: bool = True,
) -> list[RecordedEpisode]:
    episodes: list[RecordedEpisode] = []

    for episode_dir in iter_episode_dirs(raw_dataset_root):
        episode = load_recorded_episode(
            episode_dir,
            require_actions=require_actions,
        )
        if success_only and not episode.success:
            continue
        episodes.append(episode)

    if not episodes:
        raise EpisodeValidationError(
            f"No convertible episodes found in {raw_dataset_root}"
        )

    return episodes


def split_episodes(
    episodes: Iterable[RecordedEpisode],
    *,
    val_ratio: float = 0.0,
) -> tuple[list[RecordedEpisode], list[RecordedEpisode]]:
    ordered = sorted(episodes, key=lambda episode: episode.episode_id)
    if not 0.0 <= val_ratio < 1.0:
        raise ValueError("val_ratio must be in [0.0, 1.0)")
    if not ordered or val_ratio == 0.0:
        return ordered, []

    val_count = max(1, int(round(len(ordered) * val_ratio)))
    if val_count >= len(ordered):
        val_count = len(ordered) - 1
    if val_count <= 0:
        return ordered, []

    return ordered[:-val_count], ordered[-val_count:]


def read_rgb_image(path: Path) -> np.ndarray:
    with Image.open(path) as image:
        rgb = image.convert("RGB")
        return np.asarray(rgb, dtype=np.uint8)


def zero_language_embedding() -> np.ndarray:
    return np.zeros((512,), dtype=np.float32)
