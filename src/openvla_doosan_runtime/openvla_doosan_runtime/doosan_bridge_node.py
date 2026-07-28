from __future__ import annotations

import threading
import time
from typing import Optional

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
    """Service bridge for Doosan A0509. Executes at most one MoveLine at a time."""

    def __init__(self) -> None:
        super().__init__("doosan_bridge")

        self.declare_parameter("robot_namespace", "/dsr01")
        self.declare_parameter("pose_poll_hz", 10.0)
        self.declare_parameter("linear_velocity", [20.0, 10.0])
        self.declare_parameter("linear_acceleration", [40.0, 20.0])
        self.declare_parameter("motion_time_sec", 0.0)
        self.declare_parameter("reference", 0)  # DR_BASE
        self.declare_parameter("motion_timeout_sec", 10.0)
        self.declare_parameter("dry_run", False)

        self.declare_parameter("gripper_open_output_index", 1)
        self.declare_parameter("gripper_close_output_index", 2)
        self.declare_parameter("gripper_active_value", 1)
        self.declare_parameter("gripper_inactive_value", 0)
        self.declare_parameter("gripper_open_pulse_count", 2)
        self.declare_parameter("gripper_close_pulse_count", 1)
        self.declare_parameter("gripper_pulse_time_sec", 1.0)

        ns = str(self.get_parameter("robot_namespace").value).rstrip("/")
        self.get_pose_client = self.create_client(
            GetCurrentPosx,
            f"{ns}/aux_control/get_current_posx",
        )
        self.move_line_client = self.create_client(
            MoveLine,
            f"{ns}/motion/move_line",
        )
        self.stop_client = self.create_client(
            MoveStop,
            f"{ns}/motion/move_stop",
        )
        self.tool_digital_output_client = self.create_client(
            SetToolDigitalOutput,
            f"{ns}/io/set_tool_digital_output",
        )

        self.pose_publisher = self.create_publisher(
            Float64MultiArray, "/doosan/current_pose", 10
        )
        self.ready_publisher = self.create_publisher(Bool, "/vla/robot_ready", 10)
        self.done_publisher = self.create_publisher(Bool, "/vla/step_done", 10)
        self.status_publisher = self.create_publisher(String, "/doosan/status", 10)

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
        self.create_subscription(Bool, "/vla/enable", self._enable_callback, 10)
        self.create_subscription(Bool, "/vla/emergency_stop", self._stop_callback, 10)

        self.enabled = False
        self.motion_busy = False
        self.pending_gripper_open: Optional[bool] = None
        self.last_gripper_open: Optional[bool] = None
        self.last_pose = None
        self.lock = threading.Lock()

        period = 1.0 / max(1.0, float(self.get_parameter("pose_poll_hz").value))
        self.pose_timer = self.create_timer(period, self._request_pose)
        self.ready_timer = self.create_timer(0.1, self._publish_ready)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)
        self.get_logger().info(text)

    def _enable_callback(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if not self.enabled:
            self._publish_status("disabled")

    def _publish_ready(self) -> None:
        msg = Bool()
        msg.data = bool(self.enabled and not self.motion_busy and self.last_pose is not None)
        self.ready_publisher.publish(msg)

    def _request_pose(self) -> None:
        if not self.get_pose_client.service_is_ready():
            return
        request = GetCurrentPosx.Request()
        request.ref = int(self.get_parameter("reference").value)
        future = self.get_pose_client.call_async(request)
        future.add_done_callback(self._pose_response)

    def _pose_response(self, future) -> None:
        try:
            response = future.result()
            if not response.success or not response.task_pos_info:
                return
            pose = checked_array(
                response.task_pos_info[0].data[:POSE_DIM],
                POSE_DIM,
                "Doosan current pose",
            )
            self.last_pose = pose
            self.pose_publisher.publish(array_message(pose))
        except Exception as exc:
            self._publish_status(f"pose_error:{exc}")

    def _gripper_callback(self, msg: Bool) -> None:
        requested = bool(msg.data)
        with self.lock:
            if self.last_gripper_open is None or requested != self.last_gripper_open:
                self.pending_gripper_open = requested
            else:
                self.pending_gripper_open = None

    def _target_callback(self, msg: Float64MultiArray) -> None:
        if not self.enabled:
            self._publish_status("target_rejected:disabled")
            return
        if self.motion_busy:
            self._publish_status("target_rejected:motion_busy")
            return

        try:
            target = checked_array(msg.data, POSE_DIM, "target pose")
        except ValueError as exc:
            self._publish_status(f"target_rejected:{exc}")
            return

        self.motion_busy = True
        threading.Thread(
            target=self._execute_step,
            args=(target.tolist(),),
            daemon=True,
        ).start()

    def _execute_step(self, target) -> None:
        done = Bool()
        try:
            if bool(self.get_parameter("dry_run").value):
                self._execute_dry_run_step(target, done)
                return

            if not self.move_line_client.wait_for_service(timeout_sec=2.0):
                raise RuntimeError("move_line service unavailable")

            request = MoveLine.Request()
            request.pos = [float(v) for v in target]
            request.vel = [
                float(v)
                for v in self.get_parameter("linear_velocity").value
            ]
            request.acc = [
                float(v)
                for v in self.get_parameter("linear_acceleration").value
            ]
            request.time = float(self.get_parameter("motion_time_sec").value)
            request.radius = 0.0
            request.ref = int(self.get_parameter("reference").value)
            request.mode = 0  # DR_MV_MOD_ABS
            request.blend_type = 0
            request.sync_type = 0  # synchronous service motion

            self._publish_status(f"step_start:target={target}")
            future = self.move_line_client.call_async(request)
            timeout = float(self.get_parameter("motion_timeout_sec").value)
            deadline = time.monotonic() + timeout
            while not future.done():
                if time.monotonic() > deadline:
                    self._send_stop()
                    raise TimeoutError("move_line timeout")
                time.sleep(0.01)

            response = future.result()
            if response is None or not response.success:
                raise RuntimeError("move_line returned failure")

            with self.lock:
                gripper_open = self.pending_gripper_open
                self.pending_gripper_open = None
            if gripper_open is not None:
                self._set_gripper(gripper_open)
                with self.lock:
                    self.last_gripper_open = gripper_open
            else:
                self._publish_status("gripper_unchanged")

            done.data = True
            self.done_publisher.publish(done)
            self._publish_status("step_done")
        except Exception as exc:
            done.data = False
            self.done_publisher.publish(done)
            self._publish_status(f"step_failed:{exc}")
            self.get_logger().error(str(exc))
        finally:
            self.motion_busy = False

    def _execute_dry_run_step(self, target, done: Bool) -> None:
        with self.lock:
            gripper_open = self.pending_gripper_open
            self.pending_gripper_open = None
            if gripper_open is not None:
                self.last_gripper_open = gripper_open

        self._publish_status(f"dry_run_step:target={target}")
        if gripper_open is None:
            self._publish_status("dry_run_gripper_unchanged")
        else:
            self._publish_status(f"dry_run_gripper:open={gripper_open}")

        done.data = True
        self.done_publisher.publish(done)
        self._publish_status("dry_run_step_done")

    def _set_gripper(self, open_gripper: bool) -> None:
        output_index = int(
            self.get_parameter(
                "gripper_open_output_index"
                if open_gripper
                else "gripper_close_output_index"
            ).value
        )
        pulse_count = int(
            self.get_parameter(
                "gripper_open_pulse_count"
                if open_gripper
                else "gripper_close_pulse_count"
            ).value
        )
        if not 1 <= output_index <= 6:
            raise ValueError("gripper tool output index must be 1..6")
        if pulse_count < 1:
            raise ValueError("gripper pulse count must be >= 1")

        if not self.tool_digital_output_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("tool digital output service unavailable")

        active_value = int(self.get_parameter("gripper_active_value").value)
        inactive_value = int(self.get_parameter("gripper_inactive_value").value)
        pulse_time = float(self.get_parameter("gripper_pulse_time_sec").value)

        for _ in range(pulse_count):
            self._set_tool_output(output_index, active_value)
            if pulse_time > 0.0:
                time.sleep(pulse_time)
            self._set_tool_output(output_index, inactive_value)
            if pulse_time > 0.0:
                time.sleep(pulse_time)

    def _set_tool_output(self, output_index: int, value: int) -> None:
        request = SetToolDigitalOutput.Request()
        request.index = int(output_index)
        request.value = int(value)

        future = self.tool_digital_output_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("tool digital output timeout")
            time.sleep(0.01)

        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("tool digital output returned failure")

    def _stop_callback(self, msg: Bool) -> None:
        if msg.data:
            self._send_stop()
            self.enabled = False
            self.motion_busy = False
            self._publish_status("emergency_stopped")

    def _send_stop(self) -> None:
        if not self.stop_client.service_is_ready():
            return
        request = MoveStop.Request()
        # MoveStop.srv uses stop_mode in the Humble driver.
        request.stop_mode = 0
        self.stop_client.call_async(request)


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
