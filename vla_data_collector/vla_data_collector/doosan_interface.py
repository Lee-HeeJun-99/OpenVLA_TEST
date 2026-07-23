from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
import DR_init
from rclpy.node import Node


@dataclass
class TcpPose:
    values: np.ndarray

    def as_array(self) -> np.ndarray:
        return self.values.copy()


class DoosanInterface:
    def __init__(
        self,
        node: Node,
        robot_id: str = "dsr01",
        robot_model: str = "a0509",
    ) -> None:
        self.node = node
        self.robot_id = robot_id
        self.robot_model = robot_model

        # 반드시 DSR_ROBOT2 import 전에 설정
        DR_init.__dsr__id = robot_id
        DR_init.__dsr__model = robot_model
        DR_init.__dsr__node = node

        # DR_init 설정 이후 import
        from DSR_ROBOT2 import (
            DR_BASE,
            ROBOT_MODE_AUTONOMOUS,
            get_current_posx,
            movel,
            set_digital_output,
            set_robot_mode,
        )

        self._DR_BASE = DR_BASE
        self._ROBOT_MODE_AUTONOMOUS = ROBOT_MODE_AUTONOMOUS
        self._get_current_posx = get_current_posx
        self._movel = movel
        self._set_digital_output = set_digital_output
        self._set_robot_mode = set_robot_mode

        self.node.get_logger().info(
            f"Doosan interface initialized: "
            f"id={robot_id}, model={robot_model}"
        )

    def get_tcp_pose(self) -> TcpPose:
        result = self._get_current_posx(
            ref=self._DR_BASE
        )

        # Doosan API 버전에 따라
        # pose만 반환하거나 (pose, solution_space)를 반환할 수 있음
        if isinstance(result, tuple):
            pose = result[0]
        else:
            pose = result

        values = np.asarray(pose, dtype=np.float64)

        if values.shape != (6,):
            raise RuntimeError(
                f"Unexpected TCP pose shape: {values.shape}, "
                f"value={pose}"
            )

        return TcpPose(values=values)

    def move_linear(
        self,
        target_pose: Sequence[float],
        velocity_mm_s: float,
        acceleration_mm_s2: float,
    ) -> None:
        pose = list(map(float, target_pose))

        if len(pose) != 6:
            raise ValueError(
                "target_pose must contain six values"
            )

        result = self._movel(
            pose,
            vel=float(velocity_mm_s),
            acc=float(acceleration_mm_s2),
            ref=self._DR_BASE,
        )

        if result not in (0, None):
            raise RuntimeError(
                f"movel failed with result: {result}"
            )

    def set_controller_digital_output(
        self,
        index: int,
        value: int,
    ) -> None:
        index = int(index)
        value = int(value)

        if index <= 0:
            raise ValueError(
                "Digital output index must be positive"
            )

        if value not in (0, 1):
            raise ValueError(
                "Digital output value must be 0 or 1"
            )

        result = self._set_digital_output(
            index,
            value,
        )

        if result not in (0, None):
            raise RuntimeError(
                "set_digital_output failed: "
                f"index={index}, value={value}, "
                f"result={result}"
            )

    def set_autonomous_mode(self) -> None:
        result = self._set_robot_mode(
            self._ROBOT_MODE_AUTONOMOUS
        )

        if result not in (0, None):
            raise RuntimeError(
                f"set_robot_mode failed: {result}"
            )
