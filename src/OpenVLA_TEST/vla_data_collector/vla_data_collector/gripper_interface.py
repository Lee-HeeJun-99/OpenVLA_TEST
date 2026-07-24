from __future__ import annotations

import time

from rclpy.node import Node

from .doosan_interface import DoosanInterface


class DigitalGripper:
    """Parallel gripper controlled by Doosan tool digital output pulses."""

    CLOSED = 0
    OPEN = 1

    def __init__(
        self,
        node: Node,
        robot: DoosanInterface,
        open_output_index: int = 1,
        close_output_index: int = 2,
        pulse_time_sec: float = 1.0,
        initial_state: int = OPEN,
    ) -> None:
        self.node = node
        self.robot = robot
        self.open_output_index = int(open_output_index)
        self.close_output_index = int(close_output_index)
        self.pulse_time_sec = float(pulse_time_sec)
        self._state = self._validate_state(initial_state)

        self._validate_output_index(self.open_output_index)
        self._validate_output_index(self.close_output_index)
        if self.pulse_time_sec < 0:
            raise ValueError("gripper pulse_time_sec must be >= 0")

    @staticmethod
    def _validate_state(state: int) -> int:
        state = int(state)
        if state not in (DigitalGripper.CLOSED, DigitalGripper.OPEN):
            raise ValueError("gripper state must be 0 (closed) or 1 (open)")
        return state

    @staticmethod
    def _validate_output_index(index: int) -> None:
        if not 1 <= int(index) <= 6:
            raise ValueError("gripper tool output index must be 1..6")

    @property
    def state(self) -> int:
        return self._state

    def _pulse_tool_output(
        self,
        index: int,
        pulse_count: int,
        wait: bool,
    ) -> None:
        delay = self.pulse_time_sec if wait else 0.0

        for _ in range(pulse_count):
            self.robot.set_tool_digital_output(index, 1)
            if delay > 0:
                time.sleep(delay)
            self.robot.set_tool_digital_output(index, 0)
            if delay > 0:
                time.sleep(delay)

    def command(self, state: int, wait: bool = True) -> None:
        state = self._validate_state(state)
        if state == self.OPEN:
            output_index = self.open_output_index
            pulse_count = 2
            state_name = "OPEN"
        else:
            output_index = self.close_output_index
            pulse_count = 1
            state_name = "CLOSED"

        self.node.get_logger().info(
            f"Gripper {state_name}: Tool DO[{output_index}] "
            f"pulse x{pulse_count}"
        )
        self._pulse_tool_output(output_index, pulse_count, wait=wait)
        self._state = state

    def open(self, wait: bool = True) -> None:
        self.command(self.OPEN, wait=wait)

    def close(self, wait: bool = True) -> None:
        self.command(self.CLOSED, wait=wait)
