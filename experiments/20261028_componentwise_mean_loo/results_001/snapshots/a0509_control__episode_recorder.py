from __future__ import annotations

import copy
import json
import math
import queue
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from PIL import Image


_STOP = object()


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _normalize_quaternion(quaternion_wxyz: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("A pose quaternion cannot have zero norm")
    return quaternion / norm


def _quaternion_multiply(left_wxyz: np.ndarray, right_wxyz: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left_wxyz
    rw, rx, ry, rz = right_wxyz
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )


def quaternion_delta_rotvec(
    previous_wxyz: np.ndarray, next_wxyz: np.ndarray
) -> np.ndarray:
    """Return the shortest world-frame quaternion delta as a rotation vector."""
    previous = _normalize_quaternion(previous_wxyz)
    following = _normalize_quaternion(next_wxyz)
    conjugate_previous = previous * np.array([1.0, -1.0, -1.0, -1.0])
    relative = _normalize_quaternion(
        _quaternion_multiply(following, conjugate_previous)
    )
    if relative[0] < 0.0:
        relative = -relative

    vector = relative[1:]
    vector_norm = float(np.linalg.norm(vector))
    if vector_norm < 1e-10:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(vector_norm, float(relative[0]))
    return vector / vector_norm * angle


def transition_action(previous_step: Mapping[str, Any], next_step: Mapping[str, Any]) -> list[float]:
    """Build a transition-aligned pose delta and continuous gripper command."""
    previous_pose = previous_step["observation"]["end_effector_pose"]
    next_pose = next_step["observation"]["end_effector_pose"]
    translation = np.asarray(next_pose["position_m"], dtype=np.float64) - np.asarray(
        previous_pose["position_m"], dtype=np.float64
    )
    rotation = quaternion_delta_rotvec(
        np.asarray(previous_pose["quaternion_wxyz"], dtype=np.float64),
        np.asarray(next_pose["quaternion_wxyz"], dtype=np.float64),
    )
    gripper_target = float(next_step["control"]["gripper_closedness_target"])
    if not math.isfinite(gripper_target) or not 0.0 <= gripper_target <= 1.0:
        raise ValueError(f"Invalid gripper command: {gripper_target}")
    return np.concatenate([translation, rotation, [gripper_target]]).tolist()


@dataclass
class _QueuedSample:
    index: int
    images: dict[str, np.ndarray]
    step: dict[str, Any]


class EpisodeRecorder:
    """Asynchronously writes camera frames and transition-aligned OpenVLA steps."""

    def __init__(self, output_root: Path, jpeg_quality: int = 92, queue_size: int = 128):
        self.output_root = Path(output_root)
        self.jpeg_quality = int(jpeg_quality)
        self._queue: queue.Queue[Any] = queue.Queue(maxsize=queue_size)
        self._thread: threading.Thread | None = None
        self._episode_dir: Path | None = None
        self._episode_metadata: dict[str, Any] = {}
        self._submitted = 0
        self._written = 0
        self._dropped = 0
        self._error: str | None = None
        self._stop_metadata: dict[str, Any] = {}
        self._lock = threading.Lock()

    @property
    def is_recording(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def episode_dir(self) -> Path | None:
        return self._episode_dir

    @property
    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "submitted": self._submitted,
                "written": self._written,
                "dropped": self._dropped,
                "error": self._error,
                "queue_depth": self._queue.qsize(),
            }

    def set_output_root(self, output_root: Path) -> Path:
        if self.is_recording:
            raise RuntimeError("Stop the active recording before changing its path")
        resolved = Path(output_root).expanduser().resolve()
        resolved.mkdir(parents=True, exist_ok=True)
        if not resolved.is_dir():
            raise NotADirectoryError(f"Recording output is not a directory: {resolved}")
        self.output_root = resolved
        return resolved

    def start(self, metadata: Mapping[str, Any], episode_name: str | None = None) -> Path:
        if self.is_recording:
            raise RuntimeError("An episode is already being recorded")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        suffix = episode_name or "a0509_episode"
        self._episode_dir = self.output_root / f"{timestamp}_{suffix}"
        self._episode_dir.mkdir(parents=True, exist_ok=False)
        (self._episode_dir / "images").mkdir()
        (self._episode_dir / "_INCOMPLETE").write_text("recording\n", encoding="ascii")

        self._episode_metadata = _jsonable(copy.deepcopy(dict(metadata)))
        self._submitted = 0
        self._written = 0
        self._dropped = 0
        self._error = None
        self._stop_metadata = {}
        self._queue = queue.Queue(maxsize=self._queue.maxsize)
        self._thread = threading.Thread(
            target=self._writer_loop,
            name="a0509-openvla-writer",
            daemon=True,
        )
        self._thread.start()
        return self._episode_dir

    def submit(self, images: Mapping[str, np.ndarray], step: Mapping[str, Any]) -> bool:
        if not self.is_recording:
            return False

        with self._lock:
            index = self._submitted
            self._submitted += 1
        sample = _QueuedSample(
            index=index,
            images={name: np.asarray(image).copy() for name, image in images.items()},
            step=_jsonable(copy.deepcopy(dict(step))),
        )
        try:
            self._queue.put_nowait(sample)
            return True
        except queue.Full:
            with self._lock:
                self._dropped += 1
            return False

    def stop(self, success: bool, reason: str = "completed", timeout_s: float = 15.0) -> None:
        if self._thread is None:
            return
        self._stop_metadata = {"success": bool(success), "reason": str(reason)}
        self._queue.put(_STOP)
        self._thread.join(timeout=timeout_s)
        if self._thread.is_alive():
            raise TimeoutError("The episode writer did not finish before the timeout")
        self._thread = None

    def _writer_loop(self) -> None:
        assert self._episode_dir is not None
        steps_path = self._episode_dir / "steps.jsonl"
        pending: _QueuedSample | None = None
        try:
            with steps_path.open("w", encoding="utf-8", newline="\n") as steps_file:
                while True:
                    item = self._queue.get()
                    if item is _STOP:
                        break
                    if pending is not None:
                        pending.step["action"] = transition_action(pending.step, item.step)
                        self._write_sample(steps_file, pending, is_last=False)
                    pending = item

                if pending is not None:
                    pending.step["action"] = [
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        float(
                            pending.step["control"]["gripper_closedness_target"]
                        ),
                    ]
                    self._write_sample(steps_file, pending, is_last=True)

            summary = {
                **self._episode_metadata,
                **self._stop_metadata,
                "num_steps": self._written,
                "num_submitted": self._submitted,
                "num_dropped": self._dropped,
                "action_convention": {
                    "shape": [7],
                    "values": [
                        "delta_x_world_m",
                        "delta_y_world_m",
                        "delta_z_world_m",
                        "delta_rotation_vector_x_rad",
                        "delta_rotation_vector_y_rad",
                        "delta_rotation_vector_z_rad",
                        "gripper_closedness_target_0_open_1_closed_continuous",
                    ],
                    "alignment": "observation_t_to_observation_t_plus_1",
                },
            }
            (self._episode_dir / "episode.json").write_text(
                json.dumps(_jsonable(summary), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            (self._episode_dir / "_INCOMPLETE").unlink(missing_ok=True)
        except Exception as error:
            with self._lock:
                self._error = f"{type(error).__name__}: {error}"

    def _write_sample(self, steps_file: Any, sample: _QueuedSample, is_last: bool) -> None:
        assert self._episode_dir is not None
        image_paths: dict[str, str] = {}
        for camera_name, image in sample.images.items():
            camera_dir = self._episode_dir / "images" / camera_name
            camera_dir.mkdir(exist_ok=True)
            relative_path = Path("images") / camera_name / f"{sample.index:06d}.jpg"
            rgb = np.asarray(image)
            if rgb.ndim != 3 or rgb.shape[2] < 3:
                raise ValueError(f"Camera {camera_name} returned invalid shape {rgb.shape}")
            if np.issubdtype(rgb.dtype, np.floating) and float(np.nanmax(rgb)) <= 1.0:
                rgb = rgb * 255.0
            rgb = np.clip(rgb[:, :, :3], 0, 255).astype(np.uint8)
            Image.fromarray(rgb, mode="RGB").save(
                self._episode_dir / relative_path,
                format="JPEG",
                quality=self.jpeg_quality,
            )
            image_paths[camera_name] = relative_path.as_posix()

        step = sample.step
        step["step_index"] = sample.index
        step["is_first"] = sample.index == 0
        step["is_last"] = is_last
        step["is_terminal"] = is_last
        step["observation"]["images"] = image_paths
        steps_file.write(json.dumps(_jsonable(step), ensure_ascii=False) + "\n")
        steps_file.flush()
        with self._lock:
            self._written += 1
