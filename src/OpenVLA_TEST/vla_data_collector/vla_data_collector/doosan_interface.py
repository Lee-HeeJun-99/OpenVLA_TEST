from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Sequence

import numpy as np
import rclpy
from dsr_msgs2.srv import (
    GetCurrentPosx,
    MoveLine,
    SetCtrlBoxDigitalOutput,
    SetRobotMode,
    SetToolDigitalOutput,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


@dataclass
class TcpPose:
    values: np.ndarray

    def as_array(self) -> np.ndarray:
        return self.values.copy()


class DoosanInterface:
    DR_BASE = 0
    DR_MV_MOD_ABS = 0
    DR_MV_RA_DUPLICATE = 0
    ROBOT_MODE_AUTONOMOUS = 1

    def __init__(
        self,
        node: Node,
        robot_id: str = "dsr01",
        robot_model: str = "a0509",
    ) -> None:
        self.node = node
        self.robot_id = robot_id
        self.robot_model = robot_model

        service_namespace = robot_id.strip("/")
        if not service_namespace:
            raise ValueError("robot_id must not be empty")
        service_prefix = f"/{service_namespace}"

        self._callback_group = ReentrantCallbackGroup()
        self._get_current_posx = node.create_client(
            GetCurrentPosx,
            f"{service_prefix}/aux_control/get_current_posx",
            callback_group=self._callback_group,
        )
        self._move_line = node.create_client(
            MoveLine,
            f"{service_prefix}/motion/move_line",
            callback_group=self._callback_group,
        )
        self._set_digital_output = node.create_client(
            SetCtrlBoxDigitalOutput,
            f"{service_prefix}/io/set_ctrl_box_digital_output",
            callback_group=self._callback_group,
        )
        self._set_tool_digital_output = node.create_client(
            SetToolDigitalOutput,
            f"{service_prefix}/io/set_tool_digital_output",
            callback_group=self._callback_group,
        )
        self._set_robot_mode = node.create_client(
            SetRobotMode,
            f"{service_prefix}/system/set_robot_mode",
            callback_group=self._callback_group,
        )

        self.node.get_logger().info(
            f"Doosan interface initialized: "
            f"id={robot_id}, model={robot_model}"
        )

    def _call_service(
        self,
        client: Any,
        request: Any,
        service_name: str,
        timeout_sec: float,
        service_wait_sec: float = 5.0,
    ) -> Any:
        if not client.wait_for_service(timeout_sec=service_wait_sec):
            raise TimeoutError(f"Service not available: {service_name}")

        future = client.call_async(request)

        if self.node.executor is None:
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=timeout_sec,
            )
        else:
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and not future.done():
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.01)

        if not future.done():
            raise TimeoutError(f"Service call timed out: {service_name}")

        result = future.result()
        if result is None:
            raise RuntimeError(f"Service call returned no result: {service_name}")

        return result

    def get_tcp_pose(self) -> TcpPose:
        request = GetCurrentPosx.Request()
        request.ref = self.DR_BASE

        result = self._call_service(
            self._get_current_posx,
            request,
            "get_current_posx",
            timeout_sec=10.0,
        )

        if not result.success:
            raise RuntimeError("get_current_posx failed")
        if not result.task_pos_info:
            raise RuntimeError("get_current_posx returned no pose data")

        pose_data = list(result.task_pos_info[0].data)
        values = np.asarray(pose_data[:6], dtype=np.float64)

        if values.shape != (6,):
            raise RuntimeError(
                f"Unexpected TCP pose shape: {values.shape}, "
                f"value={pose_data}"
            )

        return TcpPose(values=values)

    def move_linear(
        self,
        target_pose: Sequence[float],
        velocity_mm_s: float,
        acceleration_mm_s2: float,
        angular_velocity_deg_s: float | None = None,
        angular_acceleration_deg_s2: float | None = None,
    ) -> None:
        pose = list(map(float, target_pose))

        if len(pose) != 6:
            raise ValueError(
                "target_pose must contain six values"
            )

        angular_velocity = (
            velocity_mm_s
            if angular_velocity_deg_s is None
            else angular_velocity_deg_s
        )
        angular_acceleration = (
            acceleration_mm_s2
            if angular_acceleration_deg_s2 is None
            else angular_acceleration_deg_s2
        )

        velocity = [
            float(velocity_mm_s),
            float(angular_velocity),
        ]
        acceleration = [
            float(acceleration_mm_s2),
            float(angular_acceleration),
        ]

        if any(value <= 0.0 for value in velocity):
            raise ValueError("move velocity values must be positive")
        if any(value <= 0.0 for value in acceleration):
            raise ValueError("move acceleration values must be positive")

        request = MoveLine.Request()
        request.pos = pose
        request.vel = velocity
        request.acc = acceleration
        request.time = 0.0
        request.radius = 0.0
        request.ref = self.DR_BASE
        request.mode = self.DR_MV_MOD_ABS
        request.blend_type = self.DR_MV_RA_DUPLICATE
        request.sync_type = 0

        result = self._call_service(
            self._move_line,
            request,
            "move_line",
            timeout_sec=10.0,
        )

        if not result.success:
            raise RuntimeError(
                f"move_line failed: pose={pose}, "
                f"vel={velocity}, acc={acceleration}"
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

        request = SetCtrlBoxDigitalOutput.Request()
        request.index = index
        request.value = value

        result = self._call_service(
            self._set_digital_output,
            request,
            "set_ctrl_box_digital_output",
            timeout_sec=5.0,
        )

        if not result.success:
            raise RuntimeError(
                "set_digital_output failed: "
                f"index={index}, value={value}"
            )

    def set_tool_digital_output(
        self,
        index: int,
        value: int,
    ) -> None:
        index = int(index)
        value = int(value)

        if not 1 <= index <= 6:
            raise ValueError(
                "Tool digital output index must be 1..6"
            )

        if value not in (0, 1):
            raise ValueError(
                "Tool digital output value must be 0 or 1"
            )

        request = SetToolDigitalOutput.Request()
        request.index = index
        request.value = value

        result = self._call_service(
            self._set_tool_digital_output,
            request,
            "set_tool_digital_output",
            timeout_sec=5.0,
        )

        if not result.success:
            raise RuntimeError(
                "set_tool_digital_output failed: "
                f"index={index}, value={value}"
            )

    def set_autonomous_mode(self) -> None:
        request = SetRobotMode.Request()
        request.robot_mode = self.ROBOT_MODE_AUTONOMOUS

        result = self._call_service(
            self._set_robot_mode,
            request,
            "set_robot_mode",
            timeout_sec=5.0,
        )

        if not result.success:
            raise RuntimeError(
                "set_robot_mode failed"
            )
