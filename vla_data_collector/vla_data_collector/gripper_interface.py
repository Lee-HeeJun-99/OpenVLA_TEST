from __future__ import annotations

import time

from rclpy.node import Node

from .doosan_interface import DoosanInterface


class DigitalGripper:
    """Parallel gripper controlled by one Doosan controller digital output."""

    CLOSED = 0
    OPEN = 1

    def __init__(
        self,
        node: Node,
        robot: DoosanInterface,
        output_index: int = 1,
        settle_time_sec: float = 0.6,
        initial_state: int = OPEN,
    ) -> None:
        self.node = node
        self.robot = robot
        self.output_index = int(output_index)
        self.settle_time_sec = float(settle_time_sec)
        self._state = self._validate_state(initial_state)

        if not 1 <= self.output_index <= 16:
            raise ValueError("gripper output_index must be 1..16")
        if self.settle_time_sec < 0:
            raise ValueError("gripper settle_time_sec must be >= 0")

    @staticmethod
    def _validate_state(state: int) -> int:
        state = int(state)
        if state not in (DigitalGripper.CLOSED, DigitalGripper.OPEN):
            raise ValueError("gripper state must be 0 (closed) or 1 (open)")
        return state

    @property
    def state(self) -> int:
        return self._state

    def command(self, state: int, wait: bool = True) -> None:
        state = self._validate_state(state)
        self.robot.set_controller_digital_output(self.output_index, state)
        self._state = state

        state_name = "OPEN" if state == self.OPEN else "CLOSED"
        self.node.get_logger().info(
            f"Gripper {state_name}: DO[{self.output_index}]={state}"
        )

        if wait and self.settle_time_sec > 0:
            time.sleep(self.settle_time_sec)

    def open(self, wait: bool = True) -> None:
        self.command(self.OPEN, wait=wait)

    def close(self, wait: bool = True) -> None:
        self.command(self.CLOSED, wait=wait)
