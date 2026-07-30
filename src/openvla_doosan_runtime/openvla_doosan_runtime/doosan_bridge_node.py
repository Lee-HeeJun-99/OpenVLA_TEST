from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
import rclpy
from dsr_msgs2.srv import (
    GetCurrentPosx,
    MoveLine,
    MoveStop,
    SetToolDigitalOutput,
)
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from .common import POSE_DIM, array_message, checked_array


class DoosanBridgeNode(Node):
    """
    Service bridge for the Doosan A0509.

    Execution rules:
      1. When /vla/enable becomes True, the gripper is opened first only if
         use_gripper is true.
      2. /vla/robot_ready remains False until initial gripper opening finishes
         or is skipped because use_gripper is false.
      3. Only one MoveLine operation is executed at a time.
      4. While moving, newly received targets overwrite the previous pending
         target. Only the latest target is executed next.
      5. Gripper commands are applied after the current robot movement.
      6. Emergency stop clears all pending commands.
    """

    def __init__(self) -> None:
        super().__init__("doosan_bridge")

        # ------------------------------------------------------------------
        # Robot motion parameters
        # ------------------------------------------------------------------
        self.declare_parameter("robot_namespace", "/dsr01")
        self.declare_parameter("pose_poll_hz", 20.0)

        self.declare_parameter(
            "linear_velocity",
            [20.0, 10.0],
        )
        self.declare_parameter(
            "linear_acceleration",
            [40.0, 20.0],
        )

        self.declare_parameter("motion_time_sec", 0.0)
        self.declare_parameter("reference", 0)  # DR_BASE
        self.declare_parameter("motion_timeout_sec", 10.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("use_gripper", True)

        # ------------------------------------------------------------------
        # Gripper digital output parameters
        # ------------------------------------------------------------------
        self.declare_parameter("gripper_open_output_index", 1)
        self.declare_parameter("gripper_close_output_index", 2)

        self.declare_parameter("gripper_active_value", 1)
        self.declare_parameter("gripper_inactive_value", 0)

        self.declare_parameter("gripper_open_pulse_count", 2)
        self.declare_parameter("gripper_close_pulse_count", 1)
        self.declare_parameter("gripper_pulse_time_sec", 1.0)
        self.declare_parameter("gripper_open_pulse_time_sec", -1.0)
        self.declare_parameter("gripper_close_pulse_time_sec", -1.0)

        # ------------------------------------------------------------------
        # ROS service clients
        # ------------------------------------------------------------------
        namespace = str(
            self.get_parameter("robot_namespace").value
        ).rstrip("/")

        self.get_pose_client = self.create_client(
            GetCurrentPosx,
            f"{namespace}/aux_control/get_current_posx",
        )

        self.move_line_client = self.create_client(
            MoveLine,
            f"{namespace}/motion/move_line",
        )

        self.stop_client = self.create_client(
            MoveStop,
            f"{namespace}/motion/move_stop",
        )

        self.tool_digital_output_client = self.create_client(
            SetToolDigitalOutput,
            f"{namespace}/io/set_tool_digital_output",
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.pose_publisher = self.create_publisher(
            Float64MultiArray,
            "/doosan/current_pose",
            10,
        )

        self.ready_publisher = self.create_publisher(
            Bool,
            "/vla/robot_ready",
            10,
        )

        self.done_publisher = self.create_publisher(
            Bool,
            "/vla/step_done",
            10,
        )

        self.status_publisher = self.create_publisher(
            String,
            "/doosan/status",
            10,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            Float64MultiArray,
            "/vla/target_pose",
            self._target_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/vla/gripper_open",
            self._gripper_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/vla/enable",
            self._enable_callback,
            10,
        )

        self.create_subscription(
            Bool,
            "/vla/emergency_stop",
            self._stop_callback,
            10,
        )

        # ------------------------------------------------------------------
        # Shared state
        # ------------------------------------------------------------------
        self.state_lock = threading.Lock()
        self.command_condition = threading.Condition(self.state_lock)

        self.enabled = False
        self.motion_busy = False
        self.gripper_busy = False

        self.initial_gripper_required = False
        self.initial_gripper_ready = False

        self.pending_target: Optional[np.ndarray] = None
        self.pending_gripper_open: Optional[bool] = None

        self.last_gripper_open: Optional[bool] = None
        self.last_pose: Optional[np.ndarray] = None

        self.pose_request_pending = False
        self.emergency_stop_requested = False

        self.shutdown_event = threading.Event()

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        pose_poll_hz = float(
            self.get_parameter("pose_poll_hz").value
        )

        if pose_poll_hz <= 0.0:
            raise ValueError("pose_poll_hz must be greater than zero")

        self.pose_timer = self.create_timer(
            1.0 / pose_poll_hz,
            self._request_pose,
        )

        self.ready_timer = self.create_timer(
            0.1,
            self._publish_ready,
        )

        # ------------------------------------------------------------------
        # Single command worker
        # ------------------------------------------------------------------
        self.command_worker = threading.Thread(
            target=self._command_worker_loop,
            daemon=True,
            name="doosan_command_worker",
        )
        self.command_worker.start()

        self._log_parameters()

    # ======================================================================
    # Status
    # ======================================================================

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = str(text)
        self.status_publisher.publish(msg)
        self.get_logger().info(str(text))

    def _publish_step_done(self, success: bool) -> None:
        msg = Bool()
        msg.data = bool(success)
        self.done_publisher.publish(msg)

    # ======================================================================
    # Enable and ready state
    # ======================================================================

    def _enable_callback(self, msg: Bool) -> None:
        requested_enabled = bool(msg.data)

        with self.command_condition:
            if requested_enabled == self.enabled:
                return

            if requested_enabled:
                self.enabled = True
                self.emergency_stop_requested = False

                # Remove commands left from a previous episode.
                self.pending_target = None
                self.pending_gripper_open = None

                if bool(self.get_parameter("use_gripper").value):
                    # Every gripper-enabled episode physically starts open.
                    self.initial_gripper_ready = False
                    self.initial_gripper_required = True
                else:
                    self.initial_gripper_ready = True
                    self.initial_gripper_required = False

                self.command_condition.notify_all()

            else:
                self.enabled = False

                self.pending_target = None
                self.pending_gripper_open = None

                self.initial_gripper_required = False
                self.initial_gripper_ready = False

                self.command_condition.notify_all()

        if requested_enabled:
            if bool(self.get_parameter("use_gripper").value):
                self._publish_status(
                    "enabled:initial_gripper_open_requested"
                )
            else:
                self._publish_status("enabled:gripper_disabled")
        else:
            self._publish_status("disabled")

    def _publish_ready(self) -> None:
        with self.state_lock:
            ready = bool(
                self.enabled
                and self.initial_gripper_ready
                and not self.motion_busy
                and not self.gripper_busy
                and self.last_pose is not None
                and not self.emergency_stop_requested
            )

        msg = Bool()
        msg.data = ready
        self.ready_publisher.publish(msg)

    # ======================================================================
    # Pose polling
    # ======================================================================

    def _request_pose(self) -> None:
        if not self.get_pose_client.service_is_ready():
            return

        with self.state_lock:
            if self.pose_request_pending:
                return

            self.pose_request_pending = True

        request = GetCurrentPosx.Request()
        request.ref = int(
            self.get_parameter("reference").value
        )

        try:
            future = self.get_pose_client.call_async(request)
            future.add_done_callback(self._pose_response)

        except Exception as exc:
            with self.state_lock:
                self.pose_request_pending = False

            self._publish_status(f"pose_request_error:{exc}")

    def _pose_response(self, future) -> None:
        try:
            response = future.result()

            if response is None:
                raise RuntimeError("get_current_posx returned no response")

            if not response.success:
                raise RuntimeError("get_current_posx returned failure")

            if not response.task_pos_info:
                raise RuntimeError(
                    "get_current_posx returned no task_pos_info"
                )

            pose = checked_array(
                response.task_pos_info[0].data[:POSE_DIM],
                POSE_DIM,
                "Doosan current pose",
            )

            with self.state_lock:
                self.last_pose = pose.copy()

            self.pose_publisher.publish(
                array_message(pose)
            )

        except Exception as exc:
            self._publish_status(f"pose_error:{exc}")

        finally:
            with self.state_lock:
                self.pose_request_pending = False

    # ======================================================================
    # Target and gripper input
    # ======================================================================

    def _target_callback(
        self,
        msg: Float64MultiArray,
    ) -> None:
        try:
            target = checked_array(
                msg.data,
                POSE_DIM,
                "target pose",
            )
        except ValueError as exc:
            self._publish_status(
                f"target_rejected:{exc}"
            )
            return

        with self.command_condition:
            if not self.enabled:
                reject_reason = "target_rejected:disabled"

            elif self.emergency_stop_requested:
                reject_reason = (
                    "target_rejected:emergency_stop_active"
                )

            else:
                previous_target_pending = (
                    self.pending_target is not None
                )

                self.pending_target = target.copy()
                self.command_condition.notify_all()

                if previous_target_pending:
                    accepted_status = (
                        "target_replaced_with_latest:"
                        f"target={target.tolist()}"
                    )
                elif (
                    self.motion_busy
                    or self.gripper_busy
                    or not self.initial_gripper_ready
                ):
                    accepted_status = (
                        "target_queued_latest:"
                        f"target={target.tolist()}"
                    )
                else:
                    accepted_status = (
                        "target_accepted:"
                        f"target={target.tolist()}"
                    )

                reject_reason = None

        if reject_reason is not None:
            self._publish_status(reject_reason)
        else:
            self._publish_status(accepted_status)

    def _gripper_callback(self, msg: Bool) -> None:
        requested_open = bool(msg.data)
        use_gripper = bool(
            self.get_parameter("use_gripper").value
        )

        with self.command_condition:
            if not self.enabled:
                return

            if use_gripper:
                # Keep only the most recent requested gripper state.
                self.pending_gripper_open = requested_open
            else:
                self.pending_gripper_open = None

            self.command_condition.notify_all()

        if not use_gripper:
            self._publish_status(
                f"gripper_ignored:disabled:open={requested_open}"
            )
            return

        self._publish_status(
            f"gripper_request:open={requested_open}"
        )

    # ======================================================================
    # Command worker
    # ======================================================================

    def _command_worker_loop(self) -> None:
        while not self.shutdown_event.is_set():
            operation = None
            target = None

            with self.command_condition:
                self.command_condition.wait_for(
                    lambda: (
                        self.shutdown_event.is_set()
                        or (
                            self.enabled
                            and not self.emergency_stop_requested
                            and (
                                self.initial_gripper_required
                                or (
                                    self.initial_gripper_ready
                                    and self.pending_target is not None
                                )
                            )
                        )
                    )
                )

                if self.shutdown_event.is_set():
                    return

                if (
                    not self.enabled
                    or self.emergency_stop_requested
                ):
                    continue

                if self.initial_gripper_required:
                    self.initial_gripper_required = False
                    self.gripper_busy = True
                    operation = "initial_gripper_open"

                elif (
                    self.initial_gripper_ready
                    and self.pending_target is not None
                ):
                    target = self.pending_target.copy()

                    # Remove it now. Targets received during movement will
                    # populate pending_target again and overwrite each other.
                    self.pending_target = None

                    self.motion_busy = True
                    operation = "move_step"

            if operation == "initial_gripper_open":
                self._execute_initial_gripper_open()

            elif operation == "move_step" and target is not None:
                self._execute_motion_step(target)

    # ======================================================================
    # Initial gripper
    # ======================================================================

    def _execute_initial_gripper_open(self) -> None:
        try:
            self._publish_status(
                "initial_gripper_open_start"
            )

            if not bool(
                self.get_parameter("use_gripper").value
            ):
                self._publish_status(
                    "initial_gripper_open_skipped:disabled"
                )
            elif bool(
                self.get_parameter("dry_run").value
            ):
                self._publish_status(
                    "dry_run_initial_gripper_open"
                )
            else:
                self._set_gripper(True)

            with self.command_condition:
                if (
                    not self.enabled
                    or self.emergency_stop_requested
                ):
                    raise RuntimeError(
                        "initial gripper open cancelled"
                    )

                self.last_gripper_open = True
                self.initial_gripper_ready = True

                # ActionAdapter may also publish True when the episode starts.
                # It is already satisfied by this initialization.
                if self.pending_gripper_open is True:
                    self.pending_gripper_open = None

                self.command_condition.notify_all()

            self._publish_status(
                "initial_gripper_open_done"
            )

        except Exception as exc:
            with self.command_condition:
                self.enabled = False
                self.initial_gripper_ready = False
                self.initial_gripper_required = False
                self.pending_target = None
                self.pending_gripper_open = None

                self.command_condition.notify_all()

            self._publish_status(
                f"initial_gripper_open_failed:{exc}"
            )
            self.get_logger().error(str(exc))

        finally:
            with self.command_condition:
                self.gripper_busy = False
                self.command_condition.notify_all()

    # ======================================================================
    # Motion execution
    # ======================================================================

    def _execute_motion_step(
        self,
        target: np.ndarray,
    ) -> None:
        step_success = False

        try:
            self._raise_if_cancelled()

            if bool(
                self.get_parameter("dry_run").value
            ):
                self._execute_dry_run_motion(target)
            else:
                self._execute_move_line(target)

            self._raise_if_cancelled()

            # Apply the most recent gripper request after the movement.
            self._execute_pending_gripper()

            self._raise_if_cancelled()

            step_success = True
            self._publish_step_done(True)
            self._publish_status("step_done")

        except Exception as exc:
            self._publish_step_done(False)
            self._publish_status(
                f"step_failed:{exc}"
            )
            self.get_logger().error(str(exc))

            # Motion or service failure disables the episode for safety.
            with self.command_condition:
                self.enabled = False
                self.initial_gripper_ready = False
                self.initial_gripper_required = False
                self.pending_target = None
                self.pending_gripper_open = None

                self.command_condition.notify_all()

        finally:
            with self.command_condition:
                self.motion_busy = False
                self.gripper_busy = False
                self.command_condition.notify_all()

            if step_success:
                # If another target arrived during this motion, the worker
                # condition is now satisfied and it will execute immediately.
                with self.command_condition:
                    self.command_condition.notify_all()

    def _execute_move_line(
        self,
        target: np.ndarray,
    ) -> None:
        if not self.move_line_client.wait_for_service(
            timeout_sec=2.0
        ):
            raise RuntimeError(
                "move_line service unavailable"
            )

        velocity = checked_array(
            self.get_parameter("linear_velocity").value,
            2,
            "linear_velocity",
        )

        acceleration = checked_array(
            self.get_parameter(
                "linear_acceleration"
            ).value,
            2,
            "linear_acceleration",
        )

        request = MoveLine.Request()
        request.pos = [
            float(value)
            for value in target
        ]
        request.vel = [
            float(value)
            for value in velocity
        ]
        request.acc = [
            float(value)
            for value in acceleration
        ]

        request.time = float(
            self.get_parameter("motion_time_sec").value
        )
        request.radius = 0.0
        request.ref = int(
            self.get_parameter("reference").value
        )
        request.mode = 0  # DR_MV_MOD_ABS
        request.blend_type = 0
        request.sync_type = 0  # synchronous service motion

        self._publish_status(
            f"step_start:target={target.tolist()}"
        )

        future = self.move_line_client.call_async(
            request
        )

        timeout_sec = float(
            self.get_parameter(
                "motion_timeout_sec"
            ).value
        )

        if timeout_sec <= 0.0:
            raise ValueError(
                "motion_timeout_sec must be greater than zero"
            )

        deadline = time.monotonic() + timeout_sec

        while not future.done():
            if self._execution_cancelled():
                self._send_stop()
                raise RuntimeError(
                    "move_line cancelled"
                )

            if time.monotonic() > deadline:
                self._send_stop()
                raise TimeoutError(
                    "move_line timeout"
                )

            time.sleep(0.01)

        response = future.result()

        if response is None:
            raise RuntimeError(
                "move_line returned no response"
            )

        if not response.success:
            raise RuntimeError(
                "move_line returned failure"
            )

    def _execute_dry_run_motion(
        self,
        target: np.ndarray,
    ) -> None:
        self._publish_status(
            f"dry_run_step:target={target.tolist()}"
        )

    # ======================================================================
    # Gripper execution
    # ======================================================================

    def _execute_pending_gripper(self) -> None:
        if not bool(
            self.get_parameter("use_gripper").value
        ):
            with self.command_condition:
                self.pending_gripper_open = None
                self.gripper_busy = False
                self.command_condition.notify_all()

            self._publish_status("gripper_skipped:disabled")
            return

        with self.command_condition:
            requested_open = self.pending_gripper_open
            self.pending_gripper_open = None

            if requested_open is None:
                should_execute = False

            elif (
                self.last_gripper_open is not None
                and requested_open
                == self.last_gripper_open
            ):
                should_execute = False

            else:
                should_execute = True
                self.gripper_busy = True

        if requested_open is None:
            self._publish_status(
                "gripper_unchanged"
            )
            return

        if not should_execute:
            self._publish_status(
                "gripper_unchanged:"
                f"open={requested_open}"
            )
            return

        self._raise_if_cancelled()

        if bool(
            self.get_parameter("dry_run").value
        ):
            self._publish_status(
                f"dry_run_gripper:open={requested_open}"
            )
        else:
            self._set_gripper(requested_open)

        self._raise_if_cancelled()

        with self.command_condition:
            self.last_gripper_open = requested_open
            self.gripper_busy = False
            self.command_condition.notify_all()

        self._publish_status(
            f"gripper_done:open={requested_open}"
        )

    def _set_gripper(
        self,
        open_gripper: bool,
    ) -> None:
        parameter_prefix = (
            "gripper_open"
            if open_gripper
            else "gripper_close"
        )
        opposite_prefix = (
            "gripper_close"
            if open_gripper
            else "gripper_open"
        )

        output_index = int(
            self.get_parameter(
                f"{parameter_prefix}_output_index"
            ).value
        )
        opposite_output_index = int(
            self.get_parameter(
                f"{opposite_prefix}_output_index"
            ).value
        )

        pulse_count = int(
            self.get_parameter(
                f"{parameter_prefix}_pulse_count"
            ).value
        )

        if not 1 <= output_index <= 6:
            raise ValueError(
                "gripper tool output index must be 1..6:"
                f"index={output_index}"
            )

        if not 1 <= opposite_output_index <= 6:
            raise ValueError(
                "gripper opposite tool output index must be 1..6:"
                f"index={opposite_output_index}"
            )

        if output_index == opposite_output_index:
            raise ValueError(
                "gripper open/close output indices must be different:"
                f"index={output_index}"
            )

        if pulse_count < 1:
            raise ValueError(
                "gripper pulse count must be at least 1:"
                f"count={pulse_count}"
            )

        if not self.tool_digital_output_client.wait_for_service(
            timeout_sec=2.0
        ):
            raise RuntimeError(
                "tool digital output service unavailable"
            )

        active_value = int(
            self.get_parameter(
                "gripper_active_value"
            ).value
        )

        inactive_value = int(
            self.get_parameter(
                "gripper_inactive_value"
            ).value
        )

        pulse_time = float(
            self.get_parameter(
                f"{parameter_prefix}_pulse_time_sec"
            ).value
        )

        if pulse_time < 0.0:
            pulse_time = float(
                self.get_parameter(
                    "gripper_pulse_time_sec"
                ).value
            )

        if pulse_time < 0.0:
            raise ValueError(
                "gripper pulse time must not be negative"
            )

        self._publish_status(
            "gripper_start:"
            f"open={open_gripper}, "
            f"output_index={output_index}, "
            f"opposite_output_index={opposite_output_index}, "
            f"pulse_count={pulse_count}, "
            f"pulse_time_sec={pulse_time}"
        )

        self._set_tool_output(
            opposite_output_index,
            inactive_value,
        )

        for pulse_index in range(pulse_count):
            self._raise_if_cancelled()

            self._set_tool_output(
                output_index,
                active_value,
            )

            if pulse_time > 0.0:
                self._interruptible_sleep(
                    pulse_time
                )

            self._set_tool_output(
                output_index,
                inactive_value,
            )

            if (
                pulse_time > 0.0
                and pulse_index < pulse_count - 1
            ):
                self._interruptible_sleep(
                    pulse_time
                )

    def _set_tool_output(
        self,
        output_index: int,
        value: int,
    ) -> None:
        request = SetToolDigitalOutput.Request()
        request.index = int(output_index)
        request.value = int(value)

        future = (
            self.tool_digital_output_client.call_async(
                request
            )
        )

        deadline = time.monotonic() + 5.0

        while not future.done():
            if self._execution_cancelled():
                raise RuntimeError(
                    "tool digital output cancelled"
                )

            if time.monotonic() > deadline:
                raise TimeoutError(
                    "tool digital output timeout"
                )

            time.sleep(0.01)

        response = future.result()

        if response is None:
            raise RuntimeError(
                "tool digital output returned no response"
            )

        if not response.success:
            raise RuntimeError(
                "tool digital output returned failure"
            )

    # ======================================================================
    # Emergency stop
    # ======================================================================

    def _stop_callback(self, msg: Bool) -> None:
        if not msg.data:
            return

        with self.command_condition:
            self.emergency_stop_requested = True
            self.enabled = False

            self.pending_target = None
            self.pending_gripper_open = None

            self.initial_gripper_required = False
            self.initial_gripper_ready = False

            self.command_condition.notify_all()

        self._send_stop()
        self._publish_status("emergency_stopped")

    def _send_stop(self) -> None:
        if not self.stop_client.service_is_ready():
            self.get_logger().warning(
                "move_stop service unavailable"
            )
            return

        request = MoveStop.Request()

        # MoveStop.srv in the Humble Doosan driver.
        request.stop_mode = 0

        try:
            self.stop_client.call_async(request)
        except Exception as exc:
            self.get_logger().error(
                f"move_stop request failed: {exc}"
            )

    # ======================================================================
    # Cancellation helpers
    # ======================================================================

    def _execution_cancelled(self) -> bool:
        with self.state_lock:
            return bool(
                self.shutdown_event.is_set()
                or self.emergency_stop_requested
                or not self.enabled
            )

    def _raise_if_cancelled(self) -> None:
        if self._execution_cancelled():
            raise RuntimeError(
                "execution cancelled"
            )

    def _interruptible_sleep(
        self,
        duration_sec: float,
    ) -> None:
        deadline = time.monotonic() + duration_sec

        while time.monotonic() < deadline:
            self._raise_if_cancelled()

            remaining = (
                deadline - time.monotonic()
            )
            time.sleep(
                min(0.01, max(0.0, remaining))
            )

    # ======================================================================
    # Parameter logging
    # ======================================================================

    def _log_parameters(self) -> None:
        self.get_logger().info(
            "DoosanBridge initialized:"
            f"robot_namespace="
            f"{self.get_parameter('robot_namespace').value}, "
            f"pose_poll_hz="
            f"{self.get_parameter('pose_poll_hz').value}, "
            f"linear_velocity="
            f"{self.get_parameter('linear_velocity').value}, "
            f"linear_acceleration="
            f"{self.get_parameter('linear_acceleration').value}, "
            f"motion_time_sec="
            f"{self.get_parameter('motion_time_sec').value}, "
            f"motion_timeout_sec="
            f"{self.get_parameter('motion_timeout_sec').value}, "
            f"dry_run="
            f"{self.get_parameter('dry_run').value}, "
            f"use_gripper="
            f"{self.get_parameter('use_gripper').value}, "
            f"gripper_open_output_index="
            f"{self.get_parameter('gripper_open_output_index').value}, "
            f"gripper_close_output_index="
            f"{self.get_parameter('gripper_close_output_index').value}, "
            f"gripper_open_pulse_count="
            f"{self.get_parameter('gripper_open_pulse_count').value}, "
            f"gripper_close_pulse_count="
            f"{self.get_parameter('gripper_close_pulse_count').value}, "
            f"gripper_pulse_time_sec="
            f"{self.get_parameter('gripper_pulse_time_sec').value}, "
            f"gripper_open_pulse_time_sec="
            f"{self.get_parameter('gripper_open_pulse_time_sec').value}, "
            f"gripper_close_pulse_time_sec="
            f"{self.get_parameter('gripper_close_pulse_time_sec').value}"
        )

    # ======================================================================
    # Shutdown
    # ======================================================================

    def destroy_node(self) -> bool:
        self.shutdown_event.set()

        with self.command_condition:
            self.command_condition.notify_all()

        if (
            self.command_worker.is_alive()
            and threading.current_thread()
            is not self.command_worker
        ):
            self.command_worker.join(
                timeout=1.0
            )

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = DoosanBridgeNode()

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
