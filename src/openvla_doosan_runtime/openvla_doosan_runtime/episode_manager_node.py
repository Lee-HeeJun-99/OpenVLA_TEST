from __future__ import annotations

import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class EpisodeState(str, Enum):
    IDLE = "idle"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"


class EpisodeManagerNode(Node):
    """
    Starts and stops an OpenVLA episode.

    Episode flow:
      IDLE
        -> STARTING
        -> waits for /vla/robot_ready=True
        -> RUNNING
        -> IDLE

    Behavior:
      - Publishes the instruction using TRANSIENT_LOCAL QoS.
      - Enables the robot before starting inference execution.
      - Starts duration measurement only after the robot becomes ready.
      - Stops after max_steps or max_duration_sec.
      - Allows a configurable number of consecutive invalid actions.
      - Requests emergency stop on motion failure, timeout, or startup timeout.
    """

    def __init__(self) -> None:
        super().__init__("episode_manager")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter(
            "instruction",
            "pick up the cube",
        )
        self.declare_parameter("max_steps", 30)
        self.declare_parameter("max_duration_sec", 60.0)

        self.declare_parameter(
            "robot_ready_timeout_sec",
            15.0,
        )

        self.declare_parameter(
            "invalid_action_limit",
            3,
        )

        self.declare_parameter(
            "emergency_stop_on_timeout",
            True,
        )

        self.declare_parameter(
            "emergency_stop_on_invalid_action",
            True,
        )

        # ------------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------------
        default_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        instruction_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.enable_publisher = self.create_publisher(
            Bool,
            "/vla/enable",
            default_qos,
        )

        self.instruction_publisher = self.create_publisher(
            String,
            "/vla/instruction",
            instruction_qos,
        )

        self.stop_publisher = self.create_publisher(
            Bool,
            "/vla/emergency_stop",
            default_qos,
        )

        self.status_publisher = self.create_publisher(
            String,
            "/vla/episode_status",
            default_qos,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            Bool,
            "/vla/step_done",
            self._step_callback,
            default_qos,
        )

        self.create_subscription(
            Bool,
            "/vla/action_valid",
            self._valid_callback,
            default_qos,
        )

        self.create_subscription(
            Bool,
            "/vla/robot_ready",
            self._robot_ready_callback,
            default_qos,
        )

        # ------------------------------------------------------------------
        # Services
        # ------------------------------------------------------------------
        self.create_service(
            SetBool,
            "/vla/set_episode",
            self._set_episode,
        )

        self.create_service(
            Trigger,
            "/vla/stop",
            self._stop_service,
        )

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self.state = EpisodeState.IDLE

        self.step_count = 0
        self.invalid_action_count = 0

        self.start_request_time = 0.0
        self.run_start_time = 0.0

        self.robot_ready = False

        self.timer = self.create_timer(
            0.2,
            self._watchdog,
        )

        self._log_parameters()

    # ======================================================================
    # Publishing helpers
    # ======================================================================

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = str(text)
        self.status_publisher.publish(msg)
        self.get_logger().info(str(text))

    def _set_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.enable_publisher.publish(msg)

    def _publish_instruction(self) -> None:
        instruction = String()
        instruction.data = str(
            self.get_parameter("instruction").value
        )

        self.instruction_publisher.publish(instruction)

        self._publish_status(
            f"instruction_published:{instruction.data}"
        )

    def _request_emergency_stop(
        self,
        reason: str,
    ) -> None:
        stop = Bool()
        stop.data = True
        self.stop_publisher.publish(stop)

        self._publish_status(
            f"emergency_stop_requested:{reason}"
        )

    # ======================================================================
    # Episode service
    # ======================================================================

    def _set_episode(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if request.data:
            return self._start_episode(response)

        return self._stop_episode_request(response)

    def _start_episode(
        self,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if self.state != EpisodeState.IDLE:
            response.success = False
            response.message = (
                "Episode is already active: "
                f"state={self.state.value}"
            )

            self._publish_status(
                "episode_start_rejected:"
                f"state={self.state.value}"
            )
            return response

        self.state = EpisodeState.STARTING

        self.step_count = 0
        self.invalid_action_count = 0

        self.robot_ready = False

        self.start_request_time = time.monotonic()
        self.run_start_time = 0.0

        # Publish the instruction before enabling all nodes.
        self._publish_instruction()
        self._set_enabled(True)

        self._publish_status(
            "episode_starting:waiting_for_robot_ready"
        )

        response.success = True
        response.message = (
            "OpenVLA episode initialization started"
        )
        return response

    def _stop_episode_request(
        self,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if self.state == EpisodeState.IDLE:
            response.success = True
            response.message = "Episode is already stopped"

            self._publish_status(
                "episode_stop_ignored:already_idle"
            )
            return response

        self._finish(
            reason="episode_stopped",
            emergency_stop=False,
        )

        response.success = True
        response.message = "OpenVLA episode stopped"
        return response

    # ======================================================================
    # Emergency stop service
    # ======================================================================

    def _stop_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        self._finish(
            reason="manual_emergency_stop",
            emergency_stop=True,
        )

        response.success = True
        response.message = "Emergency stop requested"
        return response

    # ======================================================================
    # Robot ready
    # ======================================================================

    def _robot_ready_callback(
        self,
        msg: Bool,
    ) -> None:
        self.robot_ready = bool(msg.data)

        if self.state != EpisodeState.STARTING:
            return

        if not self.robot_ready:
            return

        self.state = EpisodeState.RUNNING
        self.run_start_time = time.monotonic()

        self.step_count = 0
        self.invalid_action_count = 0

        self._publish_status(
            "episode_started:robot_ready"
        )

    # ======================================================================
    # Step handling
    # ======================================================================

    def _step_callback(
        self,
        msg: Bool,
    ) -> None:
        if self.state != EpisodeState.RUNNING:
            return

        if not msg.data:
            self._finish(
                reason="episode_failed:robot_step",
                emergency_stop=True,
            )
            return

        self.step_count += 1

        # A valid completed step resets consecutive invalid count.
        self.invalid_action_count = 0

        self._publish_status(
            f"step:{self.step_count}"
        )

        max_steps = int(
            self.get_parameter("max_steps").value
        )

        if (
            max_steps > 0
            and self.step_count >= max_steps
        ):
            self._finish(
                reason="episode_finished:max_steps",
                emergency_stop=False,
            )

    # ======================================================================
    # Action validity
    # ======================================================================

    def _valid_callback(
        self,
        msg: Bool,
    ) -> None:
        if self.state != EpisodeState.RUNNING:
            return

        if msg.data:
            self.invalid_action_count = 0
            return

        self.invalid_action_count += 1

        invalid_limit = int(
            self.get_parameter(
                "invalid_action_limit"
            ).value
        )

        self._publish_status(
            "invalid_action:"
            f"count={self.invalid_action_count}, "
            f"limit={invalid_limit}"
        )

        if invalid_limit <= 0:
            return

        if self.invalid_action_count < invalid_limit:
            return

        use_emergency_stop = bool(
            self.get_parameter(
                "emergency_stop_on_invalid_action"
            ).value
        )

        self._finish(
            reason=(
                "episode_failed:invalid_action_limit:"
                f"count={self.invalid_action_count}"
            ),
            emergency_stop=use_emergency_stop,
        )

    # ======================================================================
    # Watchdog
    # ======================================================================

    def _watchdog(self) -> None:
        now = time.monotonic()

        if self.state == EpisodeState.STARTING:
            self._check_startup_timeout(now)
            return

        if self.state == EpisodeState.RUNNING:
            self._check_episode_timeout(now)

    def _check_startup_timeout(
        self,
        now: float,
    ) -> None:
        timeout_sec = float(
            self.get_parameter(
                "robot_ready_timeout_sec"
            ).value
        )

        if timeout_sec <= 0.0:
            return

        elapsed = now - self.start_request_time

        if elapsed < timeout_sec:
            return

        self._finish(
            reason=(
                "episode_failed:robot_ready_timeout:"
                f"elapsed={elapsed:.3f}"
            ),
            emergency_stop=True,
        )

    def _check_episode_timeout(
        self,
        now: float,
    ) -> None:
        max_duration = float(
            self.get_parameter(
                "max_duration_sec"
            ).value
        )

        if max_duration <= 0.0:
            return

        if self.run_start_time <= 0.0:
            return

        elapsed = now - self.run_start_time

        if elapsed < max_duration:
            return

        use_emergency_stop = bool(
            self.get_parameter(
                "emergency_stop_on_timeout"
            ).value
        )

        self._finish(
            reason=(
                "episode_finished:timeout:"
                f"elapsed={elapsed:.3f}"
            ),
            emergency_stop=use_emergency_stop,
        )

    # ======================================================================
    # Finish
    # ======================================================================

    def _finish(
        self,
        reason: str,
        emergency_stop: bool,
    ) -> None:
        if self.state == EpisodeState.IDLE:
            if emergency_stop:
                self._request_emergency_stop(reason)

            self._publish_status(
                f"{reason}:already_idle"
            )
            return

        previous_state = self.state
        self.state = EpisodeState.STOPPING

        if emergency_stop:
            self._request_emergency_stop(reason)

        self._set_enabled(False)

        self.state = EpisodeState.IDLE
        self.robot_ready = False

        elapsed = self._current_elapsed_time()

        self._publish_status(
            f"{reason}:"
            f"previous_state={previous_state.value}, "
            f"steps={self.step_count}, "
            f"invalid_actions={self.invalid_action_count}, "
            f"elapsed={elapsed:.3f}"
        )

        self.step_count = 0
        self.invalid_action_count = 0
        self.start_request_time = 0.0
        self.run_start_time = 0.0

    def _current_elapsed_time(self) -> float:
        now = time.monotonic()

        if (
            self.state == EpisodeState.RUNNING
            and self.run_start_time > 0.0
        ):
            return max(
                0.0,
                now - self.run_start_time,
            )

        if self.run_start_time > 0.0:
            return max(
                0.0,
                now - self.run_start_time,
            )

        if self.start_request_time > 0.0:
            return max(
                0.0,
                now - self.start_request_time,
            )

        return 0.0

    # ======================================================================
    # Parameter logging
    # ======================================================================

    def _log_parameters(self) -> None:
        self.get_logger().info(
            "EpisodeManager initialized:"
            f"instruction="
            f"{self.get_parameter('instruction').value}, "
            f"max_steps="
            f"{self.get_parameter('max_steps').value}, "
            f"max_duration_sec="
            f"{self.get_parameter('max_duration_sec').value}, "
            f"robot_ready_timeout_sec="
            f"{self.get_parameter('robot_ready_timeout_sec').value}, "
            f"invalid_action_limit="
            f"{self.get_parameter('invalid_action_limit').value}, "
            f"emergency_stop_on_timeout="
            f"{self.get_parameter('emergency_stop_on_timeout').value}, "
            f"emergency_stop_on_invalid_action="
            f"{self.get_parameter('emergency_stop_on_invalid_action').value}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = EpisodeManagerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
