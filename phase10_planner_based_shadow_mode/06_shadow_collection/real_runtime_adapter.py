#!/usr/bin/env python3
"""ROS-free observer adapter for the Doosan runtime.

This module deliberately contains no rclpy import, publisher, service client,
action client, or robot SDK call.  It only validates offline configuration and
converts already-observed values into Phase 10 records.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Sequence

@dataclass(frozen=True)
class RemoteSafetyConfig:
    mode: str = "offline"
    shadow_mode: bool = True
    allow_robot_command: bool = False
    allow_gripper_command: bool = False
    allow_home_command: bool = False
    allow_trajectory_execution: bool = False
    allow_service_call: bool = False

    def validate(self) -> None:
        if self.mode not in {"offline", "dry-run"} or not self.shadow_mode:
            raise PermissionError("REMOTE_PREPARATION_REQUIRES_OFFLINE_SHADOW_MODE")
        unsafe = [name for name, value in vars(self).items() if name.startswith("allow_") and value]
        if unsafe:
            raise PermissionError(f"UNSAFE_CAPABILITY_ENABLED:{','.join(sorted(unsafe))}")


def load_config(path: Path) -> tuple[RemoteSafetyConfig, dict[str, Any]]:
    # Deliberately dependency-free parser for the flat ``safety`` mapping.
    # The complete YAML remains metadata and is not interpreted as executable
    # runtime configuration by this remote-preparation adapter.
    safety_values: dict[str, Any] = {}
    in_safety = False
    for original in Path(path).read_text(encoding="utf-8").splitlines():
        line = original.split("#", 1)[0].rstrip()
        if not line:
            continue
        if not line.startswith(" "):
            in_safety = line == "safety:"
            continue
        if in_safety and ":" in line:
            key, value = (part.strip() for part in line.split(":", 1))
            lowered = value.lower()
            safety_values[key] = True if lowered == "true" else False if lowered == "false" else value
    raw = {"safety": safety_values}
    safety = RemoteSafetyConfig(**safety_values)
    safety.validate()
    return safety, raw


def _finite(values: Sequence[float], length: int, label: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f"invalid {label}: expected {length} finite values")
    return result


class RealRuntimeObserverAdapter:
    """Schema converter for observations supplied by a future subscriber shell."""

    def __init__(self, config_path: Path) -> None:
        self.safety, self.config = load_config(config_path)

    def camera(self, *, frame_id: str, stamp_sec: float, clock_domain: str, image_path: str) -> dict[str, Any]:
        return {"kind": "camera", "frame_id": str(frame_id), "timestamp": float(stamp_sec),
                "clock_domain": str(clock_domain), "raw_image_path": str(image_path)}

    def measured_state(self, *, stamp_sec: float, clock_domain: str,
                       joint_position: Sequence[float], joint_velocity: Sequence[float],
                       ee_pose_mm_deg: Sequence[float], gripper_open: bool | None) -> dict[str, Any]:
        joints = [float(v) for v in joint_position]
        velocities = [float(v) for v in joint_velocity]
        if len(joints) != len(velocities) or not all(math.isfinite(v) for v in joints + velocities):
            raise ValueError("joint position/velocity mismatch or non-finite value")
        return {"kind": "state", "timestamp": float(stamp_sec), "clock_domain": str(clock_domain),
                "raw_joint_position": joints, "raw_joint_velocity": velocities,
                "raw_ee_pose_mm_deg": _finite(ee_pose_mm_deg, 6, "ee_pose_mm_deg"),
                "raw_gripper_state": None if gripper_open is None else {"open": bool(gripper_open)}}

    def observed_action(self, *, kind: str, stamp_sec: float, clock_domain: str,
                        values: Sequence[float], source: str) -> dict[str, Any]:
        if kind not in {"planner_raw_action", "planner_target", "executed_command"}:
            raise ValueError(f"unsupported observed action kind: {kind}")
        expected = 7 if kind == "planner_raw_action" else 6
        return {"kind": kind, "timestamp": float(stamp_sec), "clock_domain": str(clock_domain),
                "values": _finite(values, expected, kind), "source": str(source), "observation_only": True}
