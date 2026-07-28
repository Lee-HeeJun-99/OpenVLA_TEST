from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float64MultiArray, String

from .common import ACTION_DIM, POSE_DIM, array_message, checked_array


class ActionAdapterNode(Node):
    """
    Converts OpenVLA's 7-D action into an absolute Doosan pose.

    Expected action:
      [dx, dy, dz, dRx, dRy, dRz, gripper]

    Conventions:
      translation: meters
      rotation: radians
      gripper:
        0 = close
        1 = open

    Published topics:
      /vla/target_pose   : absolute Doosan TCP pose [mm, mm, mm, deg, deg, deg]
      /vla/gripper_open : True=open, False=close
      /vla/action_valid : whether the current action was accepted
      /vla/action_status: detailed status string

    Episode behavior:
      When /vla/enable changes from False to True, the gripper is always
      commanded open once before normal action processing begins.
    """

    def __init__(self) -> None:
        super().__init__("action_adapter")

        # ------------------------------------------------------------------
        # Unit conversion and step limiting
        # ------------------------------------------------------------------
        self.declare_parameter("translation_scale_to_mm", 1000.0)
        self.declare_parameter(
            "rotation_scale_to_deg",
            57.29577951308232,
        )

        self.declare_parameter("max_translation_step_mm", 5.0)
        self.declare_parameter("max_rotation_step_deg", 1.0)
        self.declare_parameter("apply_rotation", False)

        # ------------------------------------------------------------------
        # Gripper
        # ------------------------------------------------------------------
        self.declare_parameter("gripper_open_threshold", 0.7)
        self.declare_parameter("gripper_close_threshold", 0.3)

        # ------------------------------------------------------------------
        # Repeated action detection
        # ------------------------------------------------------------------
        self.declare_parameter("reject_repeated_actions", False)
        self.declare_parameter("repeated_action_limit", 10)
        self.declare_parameter("repeat_action_epsilon", 1.0e-5)

        # ------------------------------------------------------------------
        # Pose freshness
        # ------------------------------------------------------------------
        self.declare_parameter("max_pose_age_sec", 0.3)

        # ------------------------------------------------------------------
        # Coordinate conversion and workspace
        # ------------------------------------------------------------------
        self.declare_parameter(
            "axis_sign",
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        )

        self.declare_parameter(
            "workspace_min",
            [250.0, -400.0, 20.0, -360.0, -360.0, -360.0],
        )

        self.declare_parameter(
            "workspace_max",
            [750.0, 400.0, 700.0, 360.0, 360.0, 360.0],
        )

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.current_pose: Optional[np.ndarray] = None
        self.current_pose_time: Optional[Time] = None

        self.enabled = False

        self.last_raw_action: Optional[np.ndarray] = None
        self.raw_action_repeat_count = 0

        # Initial gripper state is always open.
        # None means no command has been sent in the current node lifecycle.
        self.last_gripper_open: Optional[bool] = None

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.target_publisher = self.create_publisher(
            Float64MultiArray,
            "/vla/target_pose",
            10,
        )

        self.gripper_publisher = self.create_publisher(
            Bool,
            "/vla/gripper_open",
            10,
        )

        self.valid_publisher = self.create_publisher(
            Bool,
            "/vla/action_valid",
            10,
        )

        self.status_publisher = self.create_publisher(
            String,
            "/vla/action_status",
            10,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
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

        self.create_subscription(
            Bool,
            "/vla/enable",
            self._enable_callback,
            10,
        )

        self._log_parameters()

    # ======================================================================
    # Callbacks
    # ======================================================================

    def _enable_callback(self, msg: Bool) -> None:
        requested_enabled = bool(msg.data)

        if requested_enabled == self.enabled:
            return

        self.enabled = requested_enabled
        self._reset_repeated_action_tracking()

        if self.enabled:
            # Every episode starts with the gripper open.
            self.last_gripper_open = None
            self._publish_gripper_command(
                gripper_open=True,
                force=True,
            )

            self._publish_status(
                "episode_enabled:initial_gripper_open"
            )

            self.get_logger().info(
                "Episode enabled. Initial gripper command: OPEN"
            )

        else:
            self._publish_status("episode_disabled")
            self.get_logger().info("Episode disabled")

    def _pose_callback(self, msg: Float64MultiArray) -> None:
        try:
            pose = checked_array(
                msg.data,
                POSE_DIM,
                "current pose",
            )

            if not np.all(np.isfinite(pose)):
                raise ValueError(
                    f"current pose contains non-finite values: {pose.tolist()}"
                )

            self.current_pose = pose
            self.current_pose_time = self.get_clock().now()

        except ValueError as exc:
            self.get_logger().error(str(exc))

    def _action_callback(self, msg: Float64MultiArray) -> None:
        if not self.enabled:
            # Do not publish an error at 10 Hz while the episode is disabled.
            return

        if self.current_pose is None:
            self._reject("action_rejected:no_current_pose")
            return

        if self.current_pose_time is None:
            self._reject("action_rejected:no_current_pose_timestamp")
            return

        try:
            self._validate_pose_age()

            action = checked_array(
                msg.data,
                ACTION_DIM,
                "raw action",
            )

            if not np.all(np.isfinite(action)):
                raise ValueError(
                    f"raw action contains non-finite values: {action.tolist()}"
                )

            repeat_count = self._update_raw_action_repeat_count(action)
            self._check_repeated_action(
                action=action,
                repeat_count=repeat_count,
            )

            signs = checked_array(
                self.get_parameter("axis_sign").value,
                POSE_DIM,
                "axis_sign",
            )

            if not np.all(np.isfinite(signs)):
                raise ValueError(
                    f"axis_sign contains non-finite values: {signs.tolist()}"
                )

            # --------------------------------------------------------------
            # Raw action → signed action
            # --------------------------------------------------------------
            signed_action = action[:POSE_DIM] * signs

            # --------------------------------------------------------------
            # Signed action → Doosan units
            # --------------------------------------------------------------
            scaled_delta = signed_action.copy()

            translation_scale = float(
                self.get_parameter(
                    "translation_scale_to_mm"
                ).value
            )

            rotation_scale = float(
                self.get_parameter(
                    "rotation_scale_to_deg"
                ).value
            )

            scaled_delta[:3] *= translation_scale
            scaled_delta[3:] *= rotation_scale

            # --------------------------------------------------------------
            # Step clipping
            # --------------------------------------------------------------
            delta = scaled_delta.copy()

            max_xyz = float(
                self.get_parameter(
                    "max_translation_step_mm"
                ).value
            )

            max_rot = float(
                self.get_parameter(
                    "max_rotation_step_deg"
                ).value
            )

            if max_xyz <= 0.0:
                raise ValueError(
                    "max_translation_step_mm must be greater than zero"
                )

            if max_rot < 0.0:
                raise ValueError(
                    "max_rotation_step_deg must not be negative"
                )

            delta[:3] = np.clip(
                delta[:3],
                -max_xyz,
                max_xyz,
            )

            delta[3:] = np.clip(
                delta[3:],
                -max_rot,
                max_rot,
            )

            if not bool(
                self.get_parameter("apply_rotation").value
            ):
                delta[3:] = 0.0

            # --------------------------------------------------------------
            # Delta pose → absolute target pose
            # --------------------------------------------------------------
            target = self.current_pose + delta

            self._validate_workspace(target)

            # --------------------------------------------------------------
            # Gripper processing
            # --------------------------------------------------------------
            gripper_value = float(action[6])
            gripper_open = self._resolve_gripper_state(gripper_value)

            # Publish only when the gripper state changes.
            gripper_command_published = self._publish_gripper_command(
                gripper_open=gripper_open,
                force=False,
            )

            # --------------------------------------------------------------
            # Publish target pose
            # --------------------------------------------------------------
            self.target_publisher.publish(
                array_message(target)
            )

            self._publish_valid(True)
            self._publish_status("action_valid")

            self.get_logger().info(
                "action_valid:"
                f"raw_action={action.tolist()}, "
                f"signed_action={signed_action.tolist()}, "
                f"scaled_delta={scaled_delta.tolist()}, "
                f"clipped_delta={delta.tolist()}, "
                f"current_pose={self.current_pose.tolist()}, "
                f"target={target.tolist()}, "
                f"gripper_value={gripper_value:.6f}, "
                f"gripper_open={gripper_open}, "
                f"gripper_command_published={gripper_command_published}, "
                f"raw_action_repeat_count={repeat_count}"
            )

        except Exception as exc:
            self._reject(f"action_rejected:{exc}")

    # ======================================================================
    # Pose validation
    # ======================================================================

    def _validate_pose_age(self) -> None:
        if self.current_pose_time is None:
            raise ValueError("current pose timestamp is unavailable")

        max_pose_age_sec = float(
            self.get_parameter("max_pose_age_sec").value
        )

        if max_pose_age_sec <= 0.0:
            return

        pose_age_sec = (
            self.get_clock().now() - self.current_pose_time
        ).nanoseconds / 1.0e9

        if pose_age_sec > max_pose_age_sec:
            raise ValueError(
                "stale_current_pose:"
                f"age={pose_age_sec:.3f}, "
                f"limit={max_pose_age_sec:.3f}"
            )

    def _validate_workspace(self, target: np.ndarray) -> None:
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

        if np.any(workspace_min >= workspace_max):
            raise ValueError(
                "workspace_min must be smaller than workspace_max:"
                f"min={workspace_min.tolist()}, "
                f"max={workspace_max.tolist()}"
            )

        below_min = target < workspace_min
        above_max = target > workspace_max

        if np.any(below_min) or np.any(above_max):
            violated_axes = np.where(
                np.logical_or(below_min, above_max)
            )[0].tolist()

            raise ValueError(
                "workspace:"
                f"target={target.tolist()}, "
                f"violated_axes={violated_axes}, "
                f"workspace_min={workspace_min.tolist()}, "
                f"workspace_max={workspace_max.tolist()}"
            )

    # ======================================================================
    # Gripper
    # ======================================================================

    def _resolve_gripper_state(self, gripper_value: float) -> bool:
        """
        Applies hysteresis to the gripper output.

        value >= open threshold:
            open

        value <= close threshold:
            close

        close threshold < value < open threshold:
            preserve previous state
        """

        open_threshold = float(
            self.get_parameter(
                "gripper_open_threshold"
            ).value
        )

        close_threshold = float(
            self.get_parameter(
                "gripper_close_threshold"
            ).value
        )

        if not 0.0 <= close_threshold < open_threshold <= 1.0:
            raise ValueError(
                "invalid gripper thresholds:"
                f"close={close_threshold}, "
                f"open={open_threshold}"
            )

        if gripper_value >= open_threshold:
            return True

        if gripper_value <= close_threshold:
            return False

        # Episode always starts open, so this fallback is normally open.
        if self.last_gripper_open is None:
            return True

        return self.last_gripper_open

    def _publish_gripper_command(
        self,
        gripper_open: bool,
        force: bool = False,
    ) -> bool:
        """
        Publishes a gripper command.

        Returns:
            True if a message was published.
            False if the command was skipped because the state did not change.
        """

        gripper_open = bool(gripper_open)

        if (
            not force
            and self.last_gripper_open is not None
            and gripper_open == self.last_gripper_open
        ):
            return False

        msg = Bool()
        msg.data = gripper_open
        self.gripper_publisher.publish(msg)

        self.last_gripper_open = gripper_open

        self.get_logger().info(
            f"gripper_command:{'OPEN' if gripper_open else 'CLOSE'}"
        )

        return True

    # ======================================================================
    # Repeated action detection
    # ======================================================================

    def _reset_repeated_action_tracking(self) -> None:
        self.last_raw_action = None
        self.raw_action_repeat_count = 0

    def _update_raw_action_repeat_count(
        self,
        action: np.ndarray,
    ) -> int:
        epsilon = float(
            self.get_parameter(
                "repeat_action_epsilon"
            ).value
        )

        if epsilon < 0.0:
            raise ValueError(
                "repeat_action_epsilon must not be negative"
            )

        if (
            self.last_raw_action is not None
            and np.allclose(
                action,
                self.last_raw_action,
                rtol=0.0,
                atol=epsilon,
            )
        ):
            self.raw_action_repeat_count += 1
        else:
            self.raw_action_repeat_count = 1

        self.last_raw_action = action.copy()

        return self.raw_action_repeat_count

    def _check_repeated_action(
        self,
        action: np.ndarray,
        repeat_count: int,
    ) -> None:
        reject_repeated = bool(
            self.get_parameter(
                "reject_repeated_actions"
            ).value
        )

        repeat_limit = int(
            self.get_parameter(
                "repeated_action_limit"
            ).value
        )

        if not reject_repeated:
            return

        if repeat_limit <= 0:
            return

        if repeat_count <= repeat_limit:
            return

        epsilon = float(
            self.get_parameter(
                "repeat_action_epsilon"
            ).value
        )

        raise ValueError(
            "repeated_raw_action:"
            f"count={repeat_count}, "
            f"limit={repeat_limit}, "
            f"epsilon={epsilon}, "
            f"action={action.tolist()}"
        )

    # ======================================================================
    # Status publishing
    # ======================================================================

    def _publish_valid(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self.valid_publisher.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = str(text)
        self.status_publisher.publish(msg)

    def _reject(self, reason: str) -> None:
        self._publish_valid(False)
        self._publish_status(reason)
        self.get_logger().error(reason)

    # ======================================================================
    # Parameter logging
    # ======================================================================

    def _log_parameters(self) -> None:
        self.get_logger().info(
            "ActionAdapter initialized:"
            f"translation_scale_to_mm="
            f"{self.get_parameter('translation_scale_to_mm').value}, "
            f"rotation_scale_to_deg="
            f"{self.get_parameter('rotation_scale_to_deg').value}, "
            f"max_translation_step_mm="
            f"{self.get_parameter('max_translation_step_mm').value}, "
            f"max_rotation_step_deg="
            f"{self.get_parameter('max_rotation_step_deg').value}, "
            f"apply_rotation="
            f"{self.get_parameter('apply_rotation').value}, "
            f"gripper_open_threshold="
            f"{self.get_parameter('gripper_open_threshold').value}, "
            f"gripper_close_threshold="
            f"{self.get_parameter('gripper_close_threshold').value}, "
            f"reject_repeated_actions="
            f"{self.get_parameter('reject_repeated_actions').value}, "
            f"max_pose_age_sec="
            f"{self.get_parameter('max_pose_age_sec').value}, "
            f"workspace_min="
            f"{self.get_parameter('workspace_min').value}, "
            f"workspace_max="
            f"{self.get_parameter('workspace_max').value}"
        )


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


if __name__ == "__main__":
    main()