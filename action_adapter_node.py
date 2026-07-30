from __future__ import annotations

import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float64MultiArray, String

from .common import ACTION_DIM, POSE_DIM, array_message, checked_array


class ActionAdapterNode(Node):
    """
    Converts OpenVLA's 7-D action into an absolute Doosan pose.

    Expected OpenVLA action:
      [dx, dy, dz, dRx, dRy, dRz, gripper]

    Units:
      translation: meters
      rotation: radians
      gripper:
        0 = close
        1 = open

    Published topics:
      /vla/target_pose    : absolute Doosan TCP pose [mm, mm, mm, deg, deg, deg]
      /vla/gripper_open  : True=open, False=close
      /vla/command_valid : whether the adapted robot command was accepted
      /vla/action_status : detailed adapter status string

    Episode behavior:
      When /vla/enable changes from False to True, the gripper is commanded
      open once before normal action processing begins.
    """

    def __init__(self) -> None:
        super().__init__("action_adapter")

        # ------------------------------------------------------------------
        # Topics
        # ------------------------------------------------------------------
        self.declare_parameter("current_pose_topic", "/doosan/current_pose")
        self.declare_parameter("raw_action_topic", "/vla/raw_action")
        self.declare_parameter("enable_topic", "/vla/enable")
        self.declare_parameter("target_pose_topic", "/vla/target_pose")
        self.declare_parameter("gripper_topic", "/vla/gripper_open")

        # Kept separate from inference validity to avoid multiple publishers
        # with different meanings on /vla/action_valid.
        self.declare_parameter("command_valid_topic", "/vla/command_valid")
        self.declare_parameter("status_topic", "/vla/action_status")

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
        self.declare_parameter("repeated_action_warn_count", 3)

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
        # Diagnostics
        # ------------------------------------------------------------------
        self.declare_parameter("log_interval_sec", 5.0)
        self.declare_parameter("slow_processing_warn_ms", 5.0)
        self.declare_parameter("log_full_action", False)

        self._load_parameters()
        self._validate_parameters()

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.current_pose: Optional[np.ndarray] = None
        self.current_pose_time: Optional[Time] = None

        self.enabled = False
        self.action_count = 0
        self.accepted_action_count = 0
        self.rejected_action_count = 0
        self.clipped_action_count = 0
        self.slow_processing_count = 0

        self.last_raw_action: Optional[np.ndarray] = None
        self.raw_action_repeat_count = 0

        self.last_target: Optional[np.ndarray] = None
        self.last_gripper_open: Optional[bool] = None

        self.log_window_start = time.monotonic()
        self.log_window_action_count = 0
        self.log_window_processing_ms = 0.0
        self.log_window_max_processing_ms = 0.0
        self.log_window_max_pose_age_ms = 0.0

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.target_publisher = self.create_publisher(
            Float64MultiArray,
            self.target_pose_topic,
            10,
        )
        self.gripper_publisher = self.create_publisher(
            Bool,
            self.gripper_topic,
            10,
        )
        self.valid_publisher = self.create_publisher(
            Bool,
            self.command_valid_topic,
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            Float64MultiArray,
            self.current_pose_topic,
            self._pose_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.raw_action_topic,
            self._action_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.enable_topic,
            self._enable_callback,
            10,
        )

        self._log_parameters()

    # ======================================================================
    # Parameter handling
    # ======================================================================

    def _load_parameters(self) -> None:
        self.current_pose_topic = str(
            self.get_parameter("current_pose_topic").value
        )
        self.raw_action_topic = str(
            self.get_parameter("raw_action_topic").value
        )
        self.enable_topic = str(
            self.get_parameter("enable_topic").value
        )
        self.target_pose_topic = str(
            self.get_parameter("target_pose_topic").value
        )
        self.gripper_topic = str(
            self.get_parameter("gripper_topic").value
        )
        self.command_valid_topic = str(
            self.get_parameter("command_valid_topic").value
        )
        self.status_topic = str(
            self.get_parameter("status_topic").value
        )

        self.translation_scale_to_mm = float(
            self.get_parameter("translation_scale_to_mm").value
        )
        self.rotation_scale_to_deg = float(
            self.get_parameter("rotation_scale_to_deg").value
        )
        self.max_translation_step_mm = float(
            self.get_parameter("max_translation_step_mm").value
        )
        self.max_rotation_step_deg = float(
            self.get_parameter("max_rotation_step_deg").value
        )
        self.apply_rotation = bool(
            self.get_parameter("apply_rotation").value
        )

        self.gripper_open_threshold = float(
            self.get_parameter("gripper_open_threshold").value
        )
        self.gripper_close_threshold = float(
            self.get_parameter("gripper_close_threshold").value
        )

        self.reject_repeated_actions = bool(
            self.get_parameter("reject_repeated_actions").value
        )
        self.repeated_action_limit = int(
            self.get_parameter("repeated_action_limit").value
        )
        self.repeat_action_epsilon = float(
            self.get_parameter("repeat_action_epsilon").value
        )
        self.repeated_action_warn_count = int(
            self.get_parameter("repeated_action_warn_count").value
        )

        self.max_pose_age_sec = float(
            self.get_parameter("max_pose_age_sec").value
        )

        self.axis_sign = checked_array(
            self.get_parameter("axis_sign").value,
            POSE_DIM,
            "axis_sign",
        )
        self.workspace_min = checked_array(
            self.get_parameter("workspace_min").value,
            POSE_DIM,
            "workspace_min",
        )
        self.workspace_max = checked_array(
            self.get_parameter("workspace_max").value,
            POSE_DIM,
            "workspace_max",
        )

        self.log_interval_sec = float(
            self.get_parameter("log_interval_sec").value
        )
        self.slow_processing_warn_ms = float(
            self.get_parameter("slow_processing_warn_ms").value
        )
        self.log_full_action = bool(
            self.get_parameter("log_full_action").value
        )

    def _validate_parameters(self) -> None:
        numeric_arrays = {
            "axis_sign": self.axis_sign,
            "workspace_min": self.workspace_min,
            "workspace_max": self.workspace_max,
        }

        for name, values in numeric_arrays.items():
            if not np.all(np.isfinite(values)):
                raise ValueError(
                    f"{name} contains non-finite values: {values.tolist()}"
                )

        if self.translation_scale_to_mm <= 0.0:
            raise ValueError(
                "translation_scale_to_mm must be greater than zero"
            )

        if self.rotation_scale_to_deg <= 0.0:
            raise ValueError(
                "rotation_scale_to_deg must be greater than zero"
            )

        if self.max_translation_step_mm <= 0.0:
            raise ValueError(
                "max_translation_step_mm must be greater than zero"
            )

        if self.max_rotation_step_deg < 0.0:
            raise ValueError(
                "max_rotation_step_deg must not be negative"
            )

        if not (
            0.0
            <= self.gripper_close_threshold
            < self.gripper_open_threshold
            <= 1.0
        ):
            raise ValueError(
                "Invalid gripper thresholds: "
                f"close={self.gripper_close_threshold}, "
                f"open={self.gripper_open_threshold}"
            )

        if self.repeat_action_epsilon < 0.0:
            raise ValueError(
                "repeat_action_epsilon must not be negative"
            )

        if self.repeated_action_limit < 0:
            raise ValueError(
                "repeated_action_limit must not be negative"
            )

        if self.repeated_action_warn_count < 0:
            raise ValueError(
                "repeated_action_warn_count must not be negative"
            )

        if self.max_pose_age_sec < 0.0:
            raise ValueError(
                "max_pose_age_sec must not be negative"
            )

        if np.any(self.workspace_min >= self.workspace_max):
            raise ValueError(
                "workspace_min must be smaller than workspace_max: "
                f"min={self.workspace_min.tolist()}, "
                f"max={self.workspace_max.tolist()}"
            )

        if self.log_interval_sec < 0.0:
            raise ValueError(
                "log_interval_sec must not be negative"
            )

        if self.slow_processing_warn_ms < 0.0:
            raise ValueError(
                "slow_processing_warn_ms must not be negative"
            )

    # ======================================================================
    # Callbacks
    # ======================================================================

    def _enable_callback(self, msg: Bool) -> None:
        requested_enabled = bool(msg.data)

        if requested_enabled == self.enabled:
            return

        self.enabled = requested_enabled
        self._reset_repeated_action_tracking()
        self.last_target = None

        if self.enabled:
            self.last_gripper_open = None
            self._publish_gripper_command(
                gripper_open=True,
                force=True,
            )
            self._publish_status(
                "episode_enabled:initial_gripper_open"
            )
            self.get_logger().info(
                "Episode enabled | initial_gripper=OPEN"
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
                    "current pose contains non-finite values: "
                    f"{pose.tolist()}"
                )

            self.current_pose = pose.copy()
            self.current_pose_time = self.get_clock().now()

        except ValueError as exc:
            self.get_logger().error(
                str(exc),
                throttle_duration_sec=2.0,
            )

    def _action_callback(self, msg: Float64MultiArray) -> None:
        if not self.enabled:
            return

        callback_start = time.perf_counter()
        self.action_count += 1

        try:
            current_pose, current_pose_time = self._snapshot_pose()
            pose_age_ms = self._validate_pose_age(current_pose_time)

            action = checked_array(
                msg.data,
                ACTION_DIM,
                "raw action",
            )

            if not np.all(np.isfinite(action)):
                raise ValueError(
                    "raw action contains non-finite values: "
                    f"{action.tolist()}"
                )

            repeat_count = self._update_raw_action_repeat_count(action)
            self._check_repeated_action(
                action=action,
                repeat_count=repeat_count,
            )

            signed_action = action[:POSE_DIM] * self.axis_sign

            scaled_delta = signed_action.copy()
            scaled_delta[:3] *= self.translation_scale_to_mm
            scaled_delta[3:] *= self.rotation_scale_to_deg

            delta, clipped_axes = self._clip_delta(scaled_delta)

            if not self.apply_rotation:
                delta[3:] = 0.0

            target = current_pose + delta
            self._validate_workspace(target)

            gripper_value = float(action[6])
            gripper_open = self._resolve_gripper_state(gripper_value)
            gripper_command_published = self._publish_gripper_command(
                gripper_open=gripper_open,
                force=False,
            )

            target_delta_l2 = (
                0.0
                if self.last_target is None
                else float(np.linalg.norm(target - self.last_target))
            )

            self.target_publisher.publish(
                array_message(target)
            )

            self.last_target = target.copy()
            self.accepted_action_count += 1

            processing_ms = (
                time.perf_counter() - callback_start
            ) * 1000.0

            self._record_diagnostics(
                processing_ms=processing_ms,
                pose_age_ms=pose_age_ms,
                clipped=bool(clipped_axes),
            )

            self._publish_valid(True)
            self._publish_status("action_valid")

            if (
                self.slow_processing_warn_ms > 0.0
                and processing_ms > self.slow_processing_warn_ms
            ):
                self.slow_processing_count += 1
                self.get_logger().warning(
                    "Slow action adaptation | "
                    f"processing_ms={processing_ms:.3f} | "
                    f"threshold_ms={self.slow_processing_warn_ms:.3f}"
                )

            if (
                clipped_axes
                or repeat_count >= self.repeated_action_warn_count > 0
            ):
                self.get_logger().warning(
                    "Action diagnostic | "
                    f"action_id={self.action_count} | "
                    f"pose_age_ms={pose_age_ms:.2f} | "
                    f"scaled_xyz_mm={np.round(scaled_delta[:3], 4).tolist()} | "
                    f"delta_xyz_mm={np.round(delta[:3], 4).tolist()} | "
                    f"clipped_axes={clipped_axes} | "
                    f"repeat_count={repeat_count} | "
                    f"target_delta_l2={target_delta_l2:.6f}"
                )

            if self.log_full_action:
                self.get_logger().info(
                    "Action accepted | "
                    f"action_id={self.action_count} | "
                    f"raw={action.tolist()} | "
                    f"signed={signed_action.tolist()} | "
                    f"scaled_delta={scaled_delta.tolist()} | "
                    f"final_delta={delta.tolist()} | "
                    f"current_pose={current_pose.tolist()} | "
                    f"target={target.tolist()} | "
                    f"gripper_value={gripper_value:.6f} | "
                    f"gripper_open={gripper_open} | "
                    f"gripper_command_published={gripper_command_published} | "
                    f"repeat_count={repeat_count} | "
                    f"processing_ms={processing_ms:.3f}"
                )

            self._log_periodic_diagnostics()

        except Exception as exc:
            processing_ms = (
                time.perf_counter() - callback_start
            ) * 1000.0
            self.rejected_action_count += 1

            self._record_diagnostics(
                processing_ms=processing_ms,
                pose_age_ms=None,
                clipped=False,
            )

            self._reject(
                f"action_rejected:{type(exc).__name__}:{exc}"
            )
            self._log_periodic_diagnostics()

    # ======================================================================
    # Pose and action processing
    # ======================================================================

    def _snapshot_pose(self) -> Tuple[np.ndarray, Time]:
        if self.current_pose is None:
            raise ValueError("no_current_pose")

        if self.current_pose_time is None:
            raise ValueError("no_current_pose_timestamp")

        return (
            self.current_pose.copy(),
            self.current_pose_time,
        )

    def _validate_pose_age(self, pose_time: Time) -> float:
        pose_age_sec = (
            self.get_clock().now() - pose_time
        ).nanoseconds / 1.0e9

        if (
            self.max_pose_age_sec > 0.0
            and pose_age_sec > self.max_pose_age_sec
        ):
            raise ValueError(
                "stale_current_pose:"
                f"age={pose_age_sec:.3f}, "
                f"limit={self.max_pose_age_sec:.3f}"
            )

        return pose_age_sec * 1000.0

    def _clip_delta(
        self,
        scaled_delta: np.ndarray,
    ) -> Tuple[np.ndarray, list[int]]:
        delta = scaled_delta.copy()

        translation_clipped = np.abs(delta[:3]) > self.max_translation_step_mm
        rotation_clipped = np.abs(delta[3:]) > self.max_rotation_step_deg

        delta[:3] = np.clip(
            delta[:3],
            -self.max_translation_step_mm,
            self.max_translation_step_mm,
        )
        delta[3:] = np.clip(
            delta[3:],
            -self.max_rotation_step_deg,
            self.max_rotation_step_deg,
        )

        clipped_mask = np.concatenate(
            [translation_clipped, rotation_clipped]
        )
        clipped_axes = np.flatnonzero(clipped_mask).astype(int).tolist()

        return delta, clipped_axes

    def _validate_workspace(self, target: np.ndarray) -> None:
        below_min = target < self.workspace_min
        above_max = target > self.workspace_max

        if np.any(below_min) or np.any(above_max):
            violated_axes = np.flatnonzero(
                np.logical_or(below_min, above_max)
            ).astype(int).tolist()

            raise ValueError(
                "workspace:"
                f"target={target.tolist()}, "
                f"violated_axes={violated_axes}"
            )

    # ======================================================================
    # Gripper
    # ======================================================================

    def _resolve_gripper_state(self, gripper_value: float) -> bool:
        if gripper_value >= self.gripper_open_threshold:
            return True

        if gripper_value <= self.gripper_close_threshold:
            return False

        if self.last_gripper_open is None:
            return True

        return self.last_gripper_open

    def _publish_gripper_command(
        self,
        gripper_open: bool,
        force: bool = False,
    ) -> bool:
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
            f"Gripper command | "
            f"state={'OPEN' if gripper_open else 'CLOSE'}"
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
        if (
            self.last_raw_action is not None
            and np.allclose(
                action,
                self.last_raw_action,
                rtol=0.0,
                atol=self.repeat_action_epsilon,
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
        if not self.reject_repeated_actions:
            return

        if self.repeated_action_limit <= 0:
            return

        if repeat_count <= self.repeated_action_limit:
            return

        raise ValueError(
            "repeated_raw_action:"
            f"count={repeat_count}, "
            f"limit={self.repeated_action_limit}, "
            f"epsilon={self.repeat_action_epsilon}, "
            f"action={action.tolist()}"
        )

    # ======================================================================
    # Diagnostics and status
    # ======================================================================

    def _record_diagnostics(
        self,
        processing_ms: float,
        pose_age_ms: Optional[float],
        clipped: bool,
    ) -> None:
        self.log_window_action_count += 1
        self.log_window_processing_ms += processing_ms
        self.log_window_max_processing_ms = max(
            self.log_window_max_processing_ms,
            processing_ms,
        )

        if pose_age_ms is not None:
            self.log_window_max_pose_age_ms = max(
                self.log_window_max_pose_age_ms,
                pose_age_ms,
            )

        if clipped:
            self.clipped_action_count += 1

    def _log_periodic_diagnostics(self) -> None:
        if self.log_interval_sec <= 0.0:
            return

        now = time.monotonic()
        elapsed_sec = now - self.log_window_start

        if elapsed_sec < self.log_interval_sec:
            return

        count = self.log_window_action_count
        average_processing_ms = (
            self.log_window_processing_ms / count
            if count > 0
            else 0.0
        )

        accepted_ratio = (
            self.accepted_action_count / self.action_count
            if self.action_count > 0
            else 0.0
        )

        clip_ratio = (
            self.clipped_action_count / self.accepted_action_count
            if self.accepted_action_count > 0
            else 0.0
        )

        self.get_logger().info(
            "ActionAdapter diagnostics | "
            f"actions_total={self.action_count} | "
            f"accepted={self.accepted_action_count} | "
            f"rejected={self.rejected_action_count} | "
            f"accepted_ratio={accepted_ratio:.3f} | "
            f"clipped={self.clipped_action_count} | "
            f"clip_ratio={clip_ratio:.3f} | "
            f"repeat_count={self.raw_action_repeat_count} | "
            f"processing_avg_ms={average_processing_ms:.3f} | "
            f"processing_max_ms={self.log_window_max_processing_ms:.3f} | "
            f"pose_age_max_ms={self.log_window_max_pose_age_ms:.2f} | "
            f"slow_processing={self.slow_processing_count}"
        )

        self.log_window_start = now
        self.log_window_action_count = 0
        self.log_window_processing_ms = 0.0
        self.log_window_max_processing_ms = 0.0
        self.log_window_max_pose_age_ms = 0.0

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

        self.get_logger().error(
            reason,
            throttle_duration_sec=1.0,
        )

    def _log_parameters(self) -> None:
        self.get_logger().info(
            "ActionAdapter ready | "
            f"raw_action_topic={self.raw_action_topic} | "
            f"target_pose_topic={self.target_pose_topic} | "
            f"command_valid_topic={self.command_valid_topic} | "
            f"translation_scale_to_mm={self.translation_scale_to_mm} | "
            f"rotation_scale_to_deg={self.rotation_scale_to_deg} | "
            f"max_translation_step_mm={self.max_translation_step_mm} | "
            f"max_rotation_step_deg={self.max_rotation_step_deg} | "
            f"apply_rotation={self.apply_rotation} | "
            f"max_pose_age_sec={self.max_pose_age_sec} | "
            f"axis_sign={self.axis_sign.tolist()} | "
            f"workspace_min={self.workspace_min.tolist()} | "
            f"workspace_max={self.workspace_max.tolist()}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[ActionAdapterNode] = None

    try:
        node = ActionAdapterNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
