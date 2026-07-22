from __future__ import annotations

import time

from rclpy.node import Node

from .doosan_interface import DoosanInterface


class DigitalGripper:
    """
    Single controller digital-output gripper.

    Hardware and dataset convention are identical:
      0 = closed
      1 = open
    """

    CLOSED = 0
    OPEN = 1

    def __init__(
        self,
        node: Node,
        robot: DoosanInterface,
        output_index: int,
        settle_time_sec: float = 0.6,
        initial_state: int = OPEN,
    ) -> None:
        self.node = node
        self.robot = robot
        self.output_index = int(output_index)
        self.settle_time_sec = float(settle_time_sec)

        if initial_state not in (self.CLOSED, self.OPEN):
            raise ValueError("initial_state must be 0 or 1")

        self._state = int(initial_state)

    @property
    def state(self) -> int:
        return self._state

    def command(self, value: int, wait: bool = True) -> None:
        if int(value) not in (self.CLOSED, self.OPEN):
            raise ValueError("gripper command must be 0(close) or 1(open)")

        self.robot.set_controller_digital_output(
            index=self.output_index,
            value=int(value),
        )
        self._state = int(value)

        state_name = "OPEN" if self._state == self.OPEN else "CLOSED"
        self.node.get_logger().info(
            f"Gripper {state_name}: DO{self.output_index}={self._state}"
        )

        if wait and self.settle_time_sec > 0:
            time.sleep(self.settle_time_sec)

    def open(self, wait: bool = True) -> None:
        self.command(self.OPEN, wait=wait)

    def close(self, wait: bool = True) -> None:
        self.command(self.CLOSED, wait=wait)
