from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from .common import ACTION_DIM, POSE_DIM, array_message, checked_array


class ActionAdapterNode(Node):
    """
    Converts OpenVLA's 7-D action into an absolute Doosan pose.

    Expected action by default:
      [dx, dy, dz, dRx, dRy, dRz, gripper]
      translation: meters
      rotation: radians
      gripper: 0=close, 1=open

    Change the YAML parameters if the RLDS transform used another convention.
    """

    def __init__(self) -> None:
        super().__init__("action_adapter")

        self.declare_parameter("translation_scale_to_mm", 1000.0)
        self.declare_parameter("rotation_scale_to_deg", 57.29577951308232)
        self.declare_parameter("max_translation_step_mm", 5.0)
        self.declare_parameter("max_rotation_step_deg", 1.0)
        self.declare_parameter("apply_rotation", False)
        self.declare_parameter("gripper_threshold", 0.5)
        self.declare_parameter("reject_repeated_actions", True)
        self.declare_parameter("repeated_action_limit", 5)
        self.declare_parameter("repeat_action_epsilon", 1.0e-12)

        self.declare_parameter("axis_sign", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter(
            "workspace_min",
            [250.0, -400.0, 20.0, -360.0, -360.0, -360.0],
        )
        self.declare_parameter(
            "workspace_max",
            [750.0, 400.0, 700.0, 360.0, 360.0, 360.0],
        )

        self.current_pose = None
        self.enabled = False
        self.last_raw_action = None
        self.raw_action_repeat_count = 0

        self.target_publisher = self.create_publisher(
            Float64MultiArray, "/vla/target_pose", 10
        )
        self.gripper_publisher = self.create_publisher(Bool, "/vla/gripper_open", 10)
        self.valid_publisher = self.create_publisher(Bool, "/vla/action_valid", 10)
        self.status_publisher = self.create_publisher(String, "/vla/action_status", 10)

        self.create_subscription(
            Float64MultiArray,
            "/doosan/current_pose",
            self._pose_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/vla/raw_action",
            self._action_callback,
            10,
        )
        self.create_subscription(Bool, "/vla/enable", self._enable_callback, 10)

    def _enable_callback(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled != self.enabled:
            self._reset_repeated_action_tracking()
        self.enabled = enabled

    def _pose_callback(self, msg: Float64MultiArray) -> None:
        try:
            self.current_pose = checked_array(msg.data, POSE_DIM, "current pose")
        except ValueError as exc:
            self.get_logger().error(str(exc))

    def _reject(self, reason: str) -> None:
        valid = Bool()
        valid.data = False
        self.valid_publisher.publish(valid)
        status = String()
        status.data = reason
        self.status_publisher.publish(status)
        self.get_logger().error(reason)

    def _action_callback(self, msg: Float64MultiArray) -> None:
        if not self.enabled:
            self._reject("action_rejected:episode_disabled")
            return
        if self.current_pose is None:
            self._reject("action_rejected:no_current_pose")
            return

        try:
            action = checked_array(msg.data, ACTION_DIM, "raw action")
            repeat_count = self._update_raw_action_repeat_count(action)
            repeat_limit = int(self.get_parameter("repeated_action_limit").value)
            if (
                bool(self.get_parameter("reject_repeated_actions").value)
                and repeat_limit > 0
                and repeat_count > repeat_limit
            ):
                self._reject(
                    "action_rejected:repeated_raw_action:"
                    f"count={repeat_count}, "
                    f"limit={repeat_limit}, "
                    f"epsilon={float(self.get_parameter('repeat_action_epsilon').value)}, "
                    f"action={action.tolist()}"
                )
                return

            signs = checked_array(
                self.get_parameter("axis_sign").value,
                POSE_DIM,
                "axis_sign",
            )
            delta = action[:6] * signs

            delta[:3] *= float(
                self.get_parameter("translation_scale_to_mm").value
            )
            delta[3:] *= float(
                self.get_parameter("rotation_scale_to_deg").value
            )

            max_xyz = float(self.get_parameter("max_translation_step_mm").value)
            max_rot = float(self.get_parameter("max_rotation_step_deg").value)
            delta[:3] = np.clip(delta[:3], -max_xyz, max_xyz)
            delta[3:] = np.clip(delta[3:], -max_rot, max_rot)

            if not bool(self.get_parameter("apply_rotation").value):
                delta[3:] = 0.0

            target = self.current_pose + delta
            workspace_min = checked_array(
                self.get_parameter("workspace_min").value,
                POSE_DIM,
                "workspace_min",
            )
            workspace_max = checked_array(
                self.get_parameter("workspace_max").value,
                POSE_DIM,
                "workspace_max",
            )
            if np.any(target < workspace_min) or np.any(target > workspace_max):
                self._reject(
                    "action_rejected:workspace:"
                    f"target={target.tolist()}"
                )
                return

            gripper = Bool()
            gripper.data = bool(
                action[6]
                >= float(self.get_parameter("gripper_threshold").value)
            )
            self.gripper_publisher.publish(gripper)
            self.target_publisher.publish(array_message(target))

            valid = Bool()
            valid.data = True
            self.valid_publisher.publish(valid)

            status = String()
            status.data = "action_valid"
            self.status_publisher.publish(status)
            self.get_logger().info(
                "action_valid:"
                f"delta_mm={delta[:3].tolist()}, "
                f"target={target.tolist()}, "
                f"gripper_open={gripper.data}, "
                f"raw_action_repeat_count={repeat_count}"
            )
        except Exception as exc:
            self._reject(f"action_rejected:{exc}")

    def _reset_repeated_action_tracking(self) -> None:
        self.last_raw_action = None
        self.raw_action_repeat_count = 0

    def _update_raw_action_repeat_count(self, action: np.ndarray) -> int:
        epsilon = float(self.get_parameter("repeat_action_epsilon").value)
        if (
            self.last_raw_action is not None
            and np.allclose(action, self.last_raw_action, rtol=0.0, atol=epsilon)
        ):
            self.raw_action_repeat_count += 1
        else:
            self.raw_action_repeat_count = 1
        self.last_raw_action = action.copy()
        return self.raw_action_repeat_count


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActionAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
