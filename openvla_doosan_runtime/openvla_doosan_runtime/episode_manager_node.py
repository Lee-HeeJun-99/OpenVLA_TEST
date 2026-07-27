\
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class EpisodeManagerNode(Node):
    """Starts/stops an episode and enforces step/time limits."""

    def __init__(self) -> None:
        super().__init__("episode_manager")

        self.declare_parameter("instruction", "pick up the cube")
        self.declare_parameter("max_steps", 30)
        self.declare_parameter("max_duration_sec", 60.0)

        self.enable_publisher = self.create_publisher(Bool, "/vla/enable", 10)
        self.instruction_publisher = self.create_publisher(
            String, "/vla/instruction", 10
        )
        self.stop_publisher = self.create_publisher(
            Bool, "/vla/emergency_stop", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/vla/episode_status", 10
        )

        self.create_subscription(Bool, "/vla/step_done", self._step_callback, 10)
        self.create_subscription(
            Bool, "/vla/action_valid", self._valid_callback, 10
        )

        self.create_service(SetBool, "/vla/set_episode", self._set_episode)
        self.create_service(Trigger, "/vla/stop", self._stop_service)

        self.active = False
        self.step_count = 0
        self.start_time = 0.0
        self.timer = self.create_timer(0.2, self._watchdog)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)

    def _set_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.enable_publisher.publish(msg)

    def _set_episode(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.active = True
            self.step_count = 0
            self.start_time = time.monotonic()

            instruction = String()
            instruction.data = str(self.get_parameter("instruction").value)
            self.instruction_publisher.publish(instruction)
            self._set_enabled(True)
            self._publish_status("episode_started")
            response.success = True
            response.message = "OpenVLA episode started"
        else:
            self._finish("episode_stopped")
            response.success = True
            response.message = "OpenVLA episode stopped"
        return response

    def _stop_service(self, request: Trigger.Request, response: Trigger.Response):
        stop = Bool()
        stop.data = True
        self.stop_publisher.publish(stop)
        self._finish("emergency_stop_requested")
        response.success = True
        response.message = "Emergency stop requested"
        return response

    def _step_callback(self, msg: Bool) -> None:
        if not self.active:
            return
        if not msg.data:
            self._finish("episode_failed:robot_step")
            return

        self.step_count += 1
        self._publish_status(f"step:{self.step_count}")
        if self.step_count >= int(self.get_parameter("max_steps").value):
            self._finish("episode_finished:max_steps")

    def _valid_callback(self, msg: Bool) -> None:
        if self.active and not msg.data:
            self._finish("episode_failed:invalid_action")

    def _watchdog(self) -> None:
        if not self.active:
            return
        elapsed = time.monotonic() - self.start_time
        if elapsed >= float(self.get_parameter("max_duration_sec").value):
            self._finish("episode_finished:timeout")

    def _finish(self, reason: str) -> None:
        self.active = False
        self._set_enabled(False)
        self._publish_status(reason)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EpisodeManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
