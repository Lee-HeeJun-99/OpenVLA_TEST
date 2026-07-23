from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from rclpy.node import Node


@dataclass(frozen=True)
class TcpPose:
    """TCP pose in the robot base frame: position [m], rotation [rad]."""

    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

    def as_array(self) -> np.ndarray:
        return np.asarray(
            [self.x, self.y, self.z, self.rx, self.ry, self.rz],
            dtype=np.float32,
        )


class DoosanInterface:
    """Minimal Doosan A0509 API wrapper for data collection."""

    def __init__(
        self,
        node: Node,
        robot_id: str = "dsr01",
        robot_model: str = "a0509",
    ) -> None:
        self.node = node
        self.robot_id = robot_id
        self.robot_model = robot_model

        try:
            import DR_init

            DR_init.__dsr__id = robot_id
            DR_init.__dsr__model = robot_model
            DR_init.__dsr__node = node

            # Import only after DR_init has been configured.
            from DSR_ROBOT2 import (
                DR_BASE,
                DR_MV_MOD_ABS,
                ROBOT_MODE_AUTONOMOUS,
                get_current_posx,
                movel,
                posx,
                set_digital_output,
                set_robot_mode,
            )
        except ImportError as exc:
            raise RuntimeError(
                "Doosan ROS2 Python API를 찾지 못했습니다. "
                "Doosan workspace의 install/setup.bash를 source했는지 확인하십시오."
            ) from exc

        self._DR_BASE = DR_BASE
        self._DR_MV_MOD_ABS = DR_MV_MOD_ABS
        self._ROBOT_MODE_AUTONOMOUS = ROBOT_MODE_AUTONOMOUS
        self._get_current_posx = get_current_posx
        self._movel = movel
        self._posx = posx
        self._set_digital_output = set_digital_output
        self._set_robot_mode = set_robot_mode

    @staticmethod
    def _check_result(operation: str, result: object) -> None:
        # Doosan API versions commonly return 0, True, or None on success.
        if result not in (0, True, None):
            raise RuntimeError(f"{operation} failed: result={result}")

    def set_autonomous_mode(self) -> None:
        result = self._set_robot_mode(self._ROBOT_MODE_AUTONOMOUS)
        self._check_result("set_robot_mode", result)

    def get_tcp_pose(self) -> TcpPose:
        result = self._get_current_posx(ref=self._DR_BASE)
        raw_pose = result[0] if isinstance(result, tuple) else result

        if raw_pose is None or len(raw_pose) < 6:
            raise RuntimeError(f"Invalid TCP pose returned: {raw_pose}")

        raw = np.asarray(raw_pose[:6], dtype=np.float64)
        if not np.isfinite(raw).all():
            raise RuntimeError(f"TCP pose contains NaN or infinity: {raw_pose}")

        position_m = raw[:3] / 1000.0
        rotation_rad = np.deg2rad(raw[3:6])

        return TcpPose(
            x=float(position_m[0]),
            y=float(position_m[1]),
            z=float(position_m[2]),
            rx=float(rotation_rad[0]),
            ry=float(rotation_rad[1]),
            rz=float(rotation_rad[2]),
        )

    def move_linear(
        self,
        target_pose: Sequence[float],
        velocity_mm_s: float = 30.0,
        acceleration_mm_s2: float = 60.0,
    ) -> None:
        target = np.asarray(target_pose, dtype=np.float64)

        if target.shape != (6,):
            raise ValueError(
                f"target_pose must have shape (6,), got {target.shape}"
            )
        if not np.isfinite(target).all():
            raise ValueError("target_pose contains NaN or infinity")
        if velocity_mm_s <= 0 or acceleration_mm_s2 <= 0:
            raise ValueError("velocity and acceleration must be positive")

        target_raw = self._posx(
            float(target[0] * 1000.0),
            float(target[1] * 1000.0),
            float(target[2] * 1000.0),
            float(np.rad2deg(target[3])),
            float(np.rad2deg(target[4])),
            float(np.rad2deg(target[5])),
        )

        result = self._movel(
            target_raw,
            vel=float(velocity_mm_s),
            acc=float(acceleration_mm_s2),
            ref=self._DR_BASE,
            mod=self._DR_MV_MOD_ABS,
        )
        self._check_result("movel", result)

    def set_controller_digital_output(self, index: int, value: int) -> None:
        """Set controller digital output. Current wiring: 0=close, 1=open."""
        index = int(index)
        value = int(value)

        if not 1 <= index <= 16:
            raise ValueError("controller digital output index must be 1..16")
        if value not in (0, 1):
            raise ValueError("digital output value must be 0 or 1")

        result = self._set_digital_output(index, value)
        self._check_result(
            f"set_digital_output(index={index}, value={value})",
            result,
        )
