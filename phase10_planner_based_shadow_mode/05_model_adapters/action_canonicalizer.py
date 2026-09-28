#!/usr/bin/env python3
"""Canonical action conversion for planner, OpenVLA, and OFT outputs.

This module is intentionally ROS-free.  It converts values for comparison but
cannot publish or execute a robot command.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Iterable, Sequence


CANONICAL_ORDER = (
    "delta_x_m",
    "delta_y_m",
    "delta_z_m",
    "delta_rotvec_x_rad",
    "delta_rotvec_y_rad",
    "delta_rotvec_z_rad",
    "gripper_closedness",
)


@dataclass(frozen=True)
class CanonicalAction:
    delta_translation_m: tuple[float, float, float]
    delta_rotation_rotvec_rad: tuple[float, float, float]
    gripper_closedness: float
    reference_frame: str
    time_horizon_seconds: float
    valid: bool = True
    invalid_reason: str | None = None
    source: str = "unknown"

    def vector(self) -> list[float]:
        return [
            *self.delta_translation_m,
            *self.delta_rotation_rotvec_rad,
            self.gripper_closedness,
        ]

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["vector"] = self.vector()
        payload["rotation_representation"] = "rotation_vector"
        return payload


def _finite_vector(values: Sequence[float], size: int = 7) -> list[float]:
    vector = [float(value) for value in values]
    if len(vector) != size:
        raise ValueError(f"expected {size} values, got {len(vector)}")
    if not all(math.isfinite(value) for value in vector):
        raise ValueError("action contains NaN or Inf")
    return vector


def canonicalize_action(
    values: Sequence[float],
    *,
    source: str,
    reference_frame: str = "robot_base_world_aligned",
    translation_unit: str = "m",
    rotation_representation: str = "rotation_vector",
    rotation_unit: str = "rad",
    gripper_semantics: str = "closedness_0_open_1_closed",
    time_horizon_seconds: float = 0.2,
    clip_gripper: bool = False,
) -> CanonicalAction:
    vector = _finite_vector(values)
    if time_horizon_seconds <= 0 or not math.isfinite(time_horizon_seconds):
        raise ValueError("time_horizon_seconds must be finite and positive")

    translation_scale = {"m": 1.0, "meter": 1.0, "mm": 0.001}.get(
        translation_unit.lower()
    )
    if translation_scale is None:
        raise ValueError(f"unsupported translation unit: {translation_unit}")
    if rotation_representation != "rotation_vector":
        raise ValueError(
            "only delta rotation vectors can be canonicalized without an "
            "explicit pose-to-delta conversion"
        )
    rotation_scale = {"rad": 1.0, "radian": 1.0, "deg": math.pi / 180.0}.get(
        rotation_unit.lower()
    )
    if rotation_scale is None:
        raise ValueError(f"unsupported rotation unit: {rotation_unit}")

    gripper = vector[6]
    if gripper_semantics == "openness_0_closed_1_open":
        gripper = 1.0 - gripper
    elif gripper_semantics != "closedness_0_open_1_closed":
        raise ValueError(f"unsupported gripper semantics: {gripper_semantics}")
    if clip_gripper:
        gripper = min(1.0, max(0.0, gripper))
    if not 0.0 <= gripper <= 1.0:
        raise ValueError(f"gripper closedness outside [0,1]: {gripper}")

    return CanonicalAction(
        delta_translation_m=tuple(value * translation_scale for value in vector[:3]),
        delta_rotation_rotvec_rad=tuple(value * rotation_scale for value in vector[3:6]),
        gripper_closedness=gripper,
        reference_frame=reference_frame,
        time_horizon_seconds=float(time_horizon_seconds),
        source=source,
    )


def canonicalize_chunk(
    actions: Iterable[Sequence[float]],
    **kwargs: Any,
) -> list[CanonicalAction]:
    return [canonicalize_action(action, **kwargs) for action in actions]


def integrated_displacement(actions: Sequence[CanonicalAction]) -> dict[str, Any]:
    if not actions:
        raise ValueError("cannot integrate an empty action sequence")
    frames = {action.reference_frame for action in actions}
    if len(frames) != 1:
        raise ValueError(f"cannot integrate mixed frames: {sorted(frames)}")
    return {
        "delta_translation_m": [
            sum(action.delta_translation_m[index] for action in actions)
            for index in range(3)
        ],
        # This is the small-delta rotvec sum used by the dataset convention.
        # It is not claimed to be exact finite-rotation composition.
        "delta_rotation_rotvec_rad_small_delta_sum": [
            sum(action.delta_rotation_rotvec_rad[index] for action in actions)
            for index in range(3)
        ],
        "time_horizon_seconds": sum(action.time_horizon_seconds for action in actions),
        "reference_frame": next(iter(frames)),
        "gripper_closedness_final": actions[-1].gripper_closedness,
    }

