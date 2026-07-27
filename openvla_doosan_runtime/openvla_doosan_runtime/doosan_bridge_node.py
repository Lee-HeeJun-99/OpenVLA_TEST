\
from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from dsr_msgs2.srv import (
    GetCurrentPosx,
    MoveLine,
    MoveStop,
    SetCtrlBoxDigitalOutput,
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

        self.declare_parameter("gripper_do_index", 1)
        # The official srv defines 0=ON, 1=OFF.
        # Your wiring convention was logical 0=close, 1=open.
        self.declare_parameter("gripper_open_service_value", 1)
        self.declare_parameter("gripper_close_service_value", 0)
        self.declare_parameter("gripper_settle_sec", 0.3)

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
        self.digital_output_client = self.create_client(
            SetCtrlBoxDigitalOutput,
            f"{ns}/io/set_ctrl_box_digital_output",
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
        self.last_pose = None
        self.lock = threading.Lock()

        period = 1.0 / max(1.0, float(self.get_parameter("pose_poll_hz").value))
        self.pose_timer = self.create_timer(period, self._request_pose)
        self.ready_timer = self.create_timer(0.1, self._publish_ready)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)

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
        self.pending_gripper_open = bool(msg.data)

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
            args=(target.tolist(), self.pending_gripper_open),
            daemon=True,
        ).start()

    def _execute_step(self, target, gripper_open: Optional[bool]) -> None:
        done = Bool()
        try:
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

            if gripper_open is not None:
                self._set_gripper(gripper_open)

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

    def _set_gripper(self, open_gripper: bool) -> None:
        if not self.digital_output_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("digital output service unavailable")

        request = SetCtrlBoxDigitalOutput.Request()
        request.index = int(self.get_parameter("gripper_do_index").value)
        request.value = int(
            self.get_parameter(
                "gripper_open_service_value"
                if open_gripper
                else "gripper_close_service_value"
            ).value
        )
        future = self.digital_output_client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("gripper output timeout")
            time.sleep(0.01)

        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("gripper output returned failure")
        time.sleep(float(self.get_parameter("gripper_settle_sec").value))

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
    finally:
        node.destroy_node()
        rclpy.shutdown()
