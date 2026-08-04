from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .doosan_interface import DoosanInterface
from .gripper_interface import DigitalGripper
from .make_actions import process_episode


class AutomaticCubeDatasetRecorder(Node):
    """
    Automatic OpenVLA dataset recorder for a fixed cube.

    Robot command convention:
      target_pose = [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]

    Saved dataset convention:
      tcp_pose = [x_m, y_m, z_m, rx_rad, ry_rad, rz_rad]
      gripper = 0 closed, 1 open

    Episode flow:
      1. Open the gripper.
      2. Move to a random start pose without recording.
      3. Record:
           random start -> alignment -> grasp
           -> gripper close transition -> lift
      4. Generate steps_with_actions.jsonl.

    To avoid the previous stationary-sample problem:
      - the timer runs at 10 Hz,
      - samples are accepted only while a motion phase is active,
      - endpoint samples are force-recorded,
      - gripper pulse waiting is not recorded repeatedly,
      - near-identical robot poses are filtered.
    """

    ROBOT_POSITION_TO_DATASET_SCALE = 0.001
    ROBOT_ROTATION_TO_DATASET_SCALE = np.pi / 180.0

    def __init__(self) -> None:
        super().__init__(
            "automatic_cube_dataset_recorder",
            namespace="dsr01",
        )

        # --------------------------------------------------------------
        # Dataset parameters
        # --------------------------------------------------------------
        self.declare_parameter(
            "image_topic",
            "/zed/zed_node/rgb/color/rect/image",
        )
        self.declare_parameter("dataset_root", "raw_dataset")
        self.declare_parameter("episode_id", "auto")
        self.declare_parameter("instruction", "pick up the cube")
        self.declare_parameter("task", "cube_pick")
        self.declare_parameter("record_frequency_hz", 10.0)
        self.declare_parameter("jpeg_quality", 95)

        # Filter stationary or duplicate motion samples.
        self.declare_parameter("min_translation_delta_m", 0.0005)
        self.declare_parameter("min_rotation_delta_rad", 0.001)

        # --------------------------------------------------------------
        # Robot parameters
        # --------------------------------------------------------------
        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "a0509")

        self.declare_parameter("move_velocity_mm_s", 80.0)
        self.declare_parameter(
            "move_angular_velocity_deg_s",
            20.0,
        )
        self.declare_parameter(
            "move_acceleration_mm_s2",
            160.0,
        )
        self.declare_parameter(
            "move_angular_acceleration_deg_s2",
            40.0,
        )

        # Slower descent is safer near the cube.
        self.declare_parameter(
            "descent_velocity_mm_s",
            70.0,
        )
        self.declare_parameter(
            "lift_velocity_mm_s",
            100.0,
        )
        self.declare_parameter(
            "preparation_velocity_mm_s",
            80.0,
        )

        # --------------------------------------------------------------
        # Workspace and fixed cube pose: millimeter / degree
        # --------------------------------------------------------------
        self.declare_parameter("x_min_mm", 428.25)
        self.declare_parameter("x_max_mm", 627.16)
        self.declare_parameter("y_min_mm", -218.94)
        self.declare_parameter("y_max_mm", 305.11)
        self.declare_parameter("start_z_mm", 400.0)

        # Reject random starts too close to the fixed cube XY.
        self.declare_parameter(
            "min_start_xy_distance_mm",
            120.0,
        )
        self.declare_parameter(
            "max_sampling_attempts",
            100,
        )
        self.declare_parameter("random_seed", 2026)

        # Measured real grasp pose:
        # [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        self.declare_parameter(
            "grasp_pose_mm_deg",
            [
                539.72,
                31.99,
                224.85,
                46.59,
                178.82,
                -131.11,
            ],
        )
        self.declare_parameter("alignment_z_mm", 320.0)
        self.declare_parameter("lift_z_mm", 350.0)

        # --------------------------------------------------------------
        # Gripper parameters
        # --------------------------------------------------------------
        self.declare_parameter(
            "gripper_open_output_index",
            1,
        )
        self.declare_parameter(
            "gripper_close_output_index",
            2,
        )
        self.declare_parameter(
            "gripper_pulse_time_sec",
            1.0,
        )

        # --------------------------------------------------------------
        # Read parameters
        # --------------------------------------------------------------
        self.image_topic = str(
            self.get_parameter("image_topic").value
        )
        self.dataset_root = Path(
            str(self.get_parameter("dataset_root").value)
        ).expanduser()
        self.configured_episode_id = str(
            self.get_parameter("episode_id").value
        )
        self.instruction = str(
            self.get_parameter("instruction").value
        )
        self.task = str(
            self.get_parameter("task").value
        )
        self.frequency = float(
            self.get_parameter("record_frequency_hz").value
        )
        self.jpeg_quality = int(
            self.get_parameter("jpeg_quality").value
        )
        self.min_translation_delta_m = float(
            self.get_parameter(
                "min_translation_delta_m"
            ).value
        )
        self.min_rotation_delta_rad = float(
            self.get_parameter(
                "min_rotation_delta_rad"
            ).value
        )

        self.move_velocity = float(
            self.get_parameter("move_velocity_mm_s").value
        )
        self.move_angular_velocity = float(
            self.get_parameter(
                "move_angular_velocity_deg_s"
            ).value
        )
        self.move_acceleration = float(
            self.get_parameter(
                "move_acceleration_mm_s2"
            ).value
        )
        self.move_angular_acceleration = float(
            self.get_parameter(
                "move_angular_acceleration_deg_s2"
            ).value
        )
        self.descent_velocity = float(
            self.get_parameter(
                "descent_velocity_mm_s"
            ).value
        )
        self.lift_velocity = float(
            self.get_parameter("lift_velocity_mm_s").value
        )
        self.preparation_velocity = float(
            self.get_parameter(
                "preparation_velocity_mm_s"
            ).value
        )

        self.x_min_mm = float(
            self.get_parameter("x_min_mm").value
        )
        self.x_max_mm = float(
            self.get_parameter("x_max_mm").value
        )
        self.y_min_mm = float(
            self.get_parameter("y_min_mm").value
        )
        self.y_max_mm = float(
            self.get_parameter("y_max_mm").value
        )
        self.start_z_mm = float(
            self.get_parameter("start_z_mm").value
        )
        self.min_start_xy_distance_mm = float(
            self.get_parameter(
                "min_start_xy_distance_mm"
            ).value
        )
        self.max_sampling_attempts = int(
            self.get_parameter(
                "max_sampling_attempts"
            ).value
        )
        self.rng = np.random.default_rng(
            int(
                self.get_parameter(
                    "random_seed"
                ).value
            )
        )

        self.grasp_pose_mm_deg = np.asarray(
            self.get_parameter(
                "grasp_pose_mm_deg"
            ).value,
            dtype=np.float64,
        )
        self.alignment_z_mm = float(
            self.get_parameter("alignment_z_mm").value
        )
        self.lift_z_mm = float(
            self.get_parameter("lift_z_mm").value
        )

        self._validate_parameters()

        # --------------------------------------------------------------
        # Interfaces
        # --------------------------------------------------------------
        self.robot = DoosanInterface(
            node=self,
            robot_id=str(
                self.get_parameter("robot_id").value
            ),
            robot_model=str(
                self.get_parameter("robot_model").value
            ),
        )
        self.gripper = DigitalGripper(
            node=self,
            robot=self.robot,
            open_output_index=int(
                self.get_parameter(
                    "gripper_open_output_index"
                ).value
            ),
            close_output_index=int(
                self.get_parameter(
                    "gripper_close_output_index"
                ).value
            ),
            pulse_time_sec=float(
                self.get_parameter(
                    "gripper_pulse_time_sec"
                ).value
            ),
            initial_state=DigitalGripper.OPEN,
        )

        # --------------------------------------------------------------
        # Image cache
        # --------------------------------------------------------------
        self.bridge = CvBridge()
        self._image_lock = threading.Lock()
        self._latest_image: Optional[np.ndarray] = None
        self._latest_image_stamp: Optional[float] = None

        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        # --------------------------------------------------------------
        # Recording state
        # --------------------------------------------------------------
        self._record_lock = threading.Lock()
        self._recording = False
        self._motion_active = False
        self._phase = "idle"
        self._force_next_sample = False

        self._episode_dir: Optional[Path] = None
        self._image_dir: Optional[Path] = None
        self._steps_path: Optional[Path] = None

        self._step_index = 0
        self._start_monotonic = 0.0
        self._last_saved_pose: Optional[np.ndarray] = None
        self._last_saved_gripper: Optional[int] = None

        self._skipped_stationary = 0
        self._pose_errors = 0
        self._image_errors = 0

        self.create_timer(
            1.0 / self.frequency,
            self._record_step,
        )

    # ==================================================================
    # Validation and pose utilities
    # ==================================================================

    def _validate_parameters(self) -> None:
        if self.frequency <= 0.0:
            raise ValueError(
                "record_frequency_hz must be positive"
            )
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError(
                "jpeg_quality must be between 1 and 100"
            )
        if self.x_min_mm >= self.x_max_mm:
            raise ValueError(
                "x_min_mm must be smaller than x_max_mm"
            )
        if self.y_min_mm >= self.y_max_mm:
            raise ValueError(
                "y_min_mm must be smaller than y_max_mm"
            )
        if self.start_z_mm > 400.0:
            raise ValueError(
                "start_z_mm must not exceed 400 mm"
            )
        if self.grasp_pose_mm_deg.shape != (6,):
            raise ValueError(
                "grasp_pose_mm_deg must contain 6 values"
            )
        if not np.isfinite(
            self.grasp_pose_mm_deg
        ).all():
            raise ValueError(
                "grasp_pose_mm_deg contains non-finite values"
            )
        if (
            self.alignment_z_mm
            <= self.grasp_pose_mm_deg[2]
        ):
            raise ValueError(
                "alignment_z_mm must be above grasp z"
            )
        if self.lift_z_mm <= self.grasp_pose_mm_deg[2]:
            raise ValueError(
                "lift_z_mm must be above grasp z"
            )

        positive_values = {
            "move_velocity_mm_s": self.move_velocity,
            "move_angular_velocity_deg_s": (
                self.move_angular_velocity
            ),
            "move_acceleration_mm_s2": (
                self.move_acceleration
            ),
            "move_angular_acceleration_deg_s2": (
                self.move_angular_acceleration
            ),
            "descent_velocity_mm_s": (
                self.descent_velocity
            ),
            "lift_velocity_mm_s": self.lift_velocity,
            "preparation_velocity_mm_s": (
                self.preparation_velocity
            ),
        }

        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(
                    f"{name} must be positive"
                )

    @classmethod
    def _convert_robot_pose_to_dataset_pose(
        cls,
        pose_mm_deg: np.ndarray,
    ) -> np.ndarray:
        pose = np.asarray(
            pose_mm_deg,
            dtype=np.float64,
        ).copy()

        if pose.shape != (6,):
            raise ValueError(
                "TCP pose must contain 6 values"
            )

        pose[:3] *= cls.ROBOT_POSITION_TO_DATASET_SCALE
        pose[3:] *= cls.ROBOT_ROTATION_TO_DATASET_SCALE
        return pose

    def _sample_start_pose(self) -> np.ndarray:
        cube_xy = self.grasp_pose_mm_deg[:2]

        for _ in range(self.max_sampling_attempts):
            x = float(
                self.rng.uniform(
                    self.x_min_mm,
                    self.x_max_mm,
                )
            )
            y = float(
                self.rng.uniform(
                    self.y_min_mm,
                    self.y_max_mm,
                )
            )

            distance = float(
                np.linalg.norm(
                    np.asarray([x, y]) - cube_xy
                )
            )

            if distance < self.min_start_xy_distance_mm:
                continue

            pose = self.grasp_pose_mm_deg.copy()
            pose[0] = x
            pose[1] = y
            pose[2] = self.start_z_mm
            return pose

        raise RuntimeError(
            "Unable to sample a valid random start pose. "
            "Reduce min_start_xy_distance_mm."
        )

    def _build_waypoints(
        self,
        start_pose: np.ndarray,
    ) -> dict[str, np.ndarray]:
        alignment_pose = self.grasp_pose_mm_deg.copy()
        alignment_pose[2] = self.alignment_z_mm

        grasp_pose = self.grasp_pose_mm_deg.copy()

        lift_pose = self.grasp_pose_mm_deg.copy()
        lift_pose[2] = self.lift_z_mm

        return {
            "start": start_pose,
            "alignment": alignment_pose,
            "grasp": grasp_pose,
            "lift": lift_pose,
        }

    # ==================================================================
    # Image handling
    # ==================================================================

    def _image_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
            stamp = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9
            )

            with self._image_lock:
                self._latest_image = image.copy()
                self._latest_image_stamp = stamp

        except Exception as exc:
            self.get_logger().error(
                f"Image conversion failed: {exc}"
            )

    def wait_for_first_image(
        self,
        timeout_sec: float = 15.0,
    ) -> None:
        deadline = time.monotonic() + timeout_sec

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            with self._image_lock:
                if self._latest_image is not None:
                    self.get_logger().info(
                        f"Receiving ZED image: "
                        f"{self.image_topic}"
                    )
                    return

            time.sleep(0.05)

        raise TimeoutError(
            f"No image received from {self.image_topic}"
        )

    # ==================================================================
    # Episode path and metadata
    # ==================================================================

    def _resolve_episode_id(self) -> str:
        configured = self.configured_episode_id.strip()

        if configured and configured.lower() != "auto":
            return configured

        self.dataset_root.mkdir(
            parents=True,
            exist_ok=True,
        )

        used: list[int] = []

        for path in self.dataset_root.glob("episode_*"):
            suffix = path.name.removeprefix("episode_")
            if suffix.isdigit():
                used.append(int(suffix))

        return f"episode_{max(used, default=0) + 1:06d}"

    def _prepare_episode_directory(self) -> str:
        episode_id = self._resolve_episode_id()

        self._episode_dir = (
            self.dataset_root / episode_id
        )
        self._image_dir = (
            self._episode_dir / "images"
        )
        self._steps_path = (
            self._episode_dir / "steps.jsonl"
        )

        if self._episode_dir.exists():
            raise FileExistsError(
                f"Episode directory already exists: "
                f"{self._episode_dir}"
            )

        self._image_dir.mkdir(
            parents=True,
            exist_ok=False,
        )
        return episode_id

    # ==================================================================
    # Recording control
    # ==================================================================

    def _begin_recording(self) -> None:
        with self._record_lock:
            self._step_index = 0
            self._start_monotonic = time.monotonic()
            self._last_saved_pose = None
            self._last_saved_gripper = None
            self._skipped_stationary = 0
            self._pose_errors = 0
            self._image_errors = 0
            self._recording = True
            self._motion_active = False
            self._phase = "start"
            self._force_next_sample = True

    def _end_recording(self) -> None:
        with self._record_lock:
            self._recording = False
            self._motion_active = False
            self._phase = "idle"
            self._force_next_sample = False

    def _set_phase(
        self,
        phase: str,
        motion_active: bool,
        force_next_sample: bool = False,
    ) -> None:
        with self._record_lock:
            self._phase = str(phase)
            self._motion_active = bool(motion_active)
            if force_next_sample:
                self._force_next_sample = True

    def _force_record_now(
        self,
        phase: str,
    ) -> None:
        self._set_phase(
            phase=phase,
            motion_active=True,
            force_next_sample=True,
        )
        self._record_step()

    def _record_step(self) -> None:
        with self._record_lock:
            recording = self._recording
            motion_active = self._motion_active
            force_sample = self._force_next_sample
            phase = self._phase

            if force_sample:
                self._force_next_sample = False

        if not recording:
            return

        if not motion_active and not force_sample:
            return

        if (
            self._image_dir is None
            or self._steps_path is None
        ):
            return

        with self._image_lock:
            if self._latest_image is None:
                return

            image = self._latest_image.copy()
            image_stamp = self._latest_image_stamp

        try:
            robot_pose_mm_deg = (
                self.robot.get_tcp_pose().as_array()
            )
            tcp_pose = (
                self._convert_robot_pose_to_dataset_pose(
                    robot_pose_mm_deg
                )
            )

        except Exception as exc:
            self._pose_errors += 1
            self.get_logger().error(
                f"TCP pose read failed: {exc}"
            )
            return

        gripper_state = int(self.gripper.state)

        if (
            not force_sample
            and self._last_saved_pose is not None
            and self._last_saved_gripper == gripper_state
        ):
            delta = tcp_pose - self._last_saved_pose
            translation_delta = float(
                np.linalg.norm(delta[:3])
            )
            rotation_delta = float(
                np.linalg.norm(delta[3:])
            )

            if (
                translation_delta
                < self.min_translation_delta_m
                and rotation_delta
                < self.min_rotation_delta_rad
            ):
                self._skipped_stationary += 1
                return

        index = self._step_index
        image_name = f"{index:06d}.jpg"
        image_path = self._image_dir / image_name

        saved = cv2.imwrite(
            str(image_path),
            image,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                self.jpeg_quality,
            ],
        )

        if not saved:
            self._image_errors += 1
            self.get_logger().error(
                f"Failed to save image: {image_path}"
            )
            return

        record = {
            "step_index": index,
            "timestamp": (
                time.monotonic()
                - self._start_monotonic
            ),
            "image_timestamp": image_stamp,
            "image": f"images/{image_name}",
            "tcp_pose": (
                tcp_pose.astype(float).tolist()
            ),
            "gripper": gripper_state,
            "phase": phase,
        }

        try:
            with self._steps_path.open(
                "a",
                encoding="utf-8",
            ) as file:
                file.write(
                    json.dumps(
                        record,
                        ensure_ascii=False,
                    )
                    + "\n"
                )

        except Exception:
            image_path.unlink(missing_ok=True)
            raise

        self._last_saved_pose = tcp_pose.copy()
        self._last_saved_gripper = gripper_state
        self._step_index += 1

    # ==================================================================
    # Robot motion
    # ==================================================================

    def _move_linear_recorded(
        self,
        name: str,
        target_pose_mm_deg: np.ndarray,
        velocity_mm_s: float,
    ) -> None:
        self._set_phase(
            phase=name,
            motion_active=True,
            force_next_sample=True,
        )

        self.get_logger().info(
            f"Moving: {name}, "
            f"pose={np.round(target_pose_mm_deg, 3).tolist()}, "
            f"vel={velocity_mm_s:.1f}"
        )

        try:
            # IMPORTANT:
            # target_pose is intentionally mm/degree, matching the
            # previously working DatasetRecorder.
            self.robot.move_linear(
                target_pose=(
                    target_pose_mm_deg.astype(float).tolist()
                ),
                velocity_mm_s=float(velocity_mm_s),
                acceleration_mm_s2=(
                    self.move_acceleration
                ),
                angular_velocity_deg_s=(
                    self.move_angular_velocity
                ),
                angular_acceleration_deg_s2=(
                    self.move_angular_acceleration
                ),
            )

        finally:
            self._set_phase(
                phase=name,
                motion_active=False,
            )

        # Preserve the endpoint exactly once.
        self._force_record_now(
            phase=f"{name}_end"
        )
        self._set_phase(
            phase=f"{name}_end",
            motion_active=False,
        )

        self.get_logger().info(
            f"Reached: {name}"
        )

    # ==================================================================
    # Episode execution
    # ==================================================================

    def run_episode(self) -> None:
        self.wait_for_first_image()

        self.get_logger().info(
            "Setting robot mode: autonomous"
        )
        self.robot.set_autonomous_mode()
        self.get_logger().info(
            "Robot mode ready: autonomous"
        )

        episode_id = self._prepare_episode_directory()
        start_pose = self._sample_start_pose()
        waypoints = self._build_waypoints(start_pose)

        print()
        print(f"Episode: {episode_id}")
        print(
            "Random start [mm, deg]: "
            f"{np.round(start_pose, 2).tolist()}"
        )
        print(
            "Route: start -> alignment -> grasp "
            "-> close -> lift"
        )
        input(
            "작업 공간과 비상 정지를 확인한 뒤 "
            "Enter를 누르세요: "
        )

        success = False
        failure_reason = ""
        started_at = time.monotonic()

        try:
            # ----------------------------------------------------------
            # Preparation: not recorded
            # ----------------------------------------------------------
            self.gripper.command(
                DigitalGripper.OPEN,
                wait=True,
            )

            self.get_logger().info(
                "Moving to random start without recording"
            )
            self.robot.move_linear(
                target_pose=(
                    waypoints["start"]
                    .astype(float)
                    .tolist()
                ),
                velocity_mm_s=(
                    self.preparation_velocity
                ),
                acceleration_mm_s2=(
                    self.move_acceleration
                ),
                angular_velocity_deg_s=(
                    self.move_angular_velocity
                ),
                angular_acceleration_deg_s2=(
                    self.move_angular_acceleration
                ),
            )

            # ----------------------------------------------------------
            # Recorded task
            # ----------------------------------------------------------
            self._begin_recording()

            # Save the initial observation once.
            self._force_record_now("start")
            self._set_phase(
                phase="start",
                motion_active=False,
            )

            self._move_linear_recorded(
                name="alignment",
                target_pose_mm_deg=(
                    waypoints["alignment"]
                ),
                velocity_mm_s=self.move_velocity,
            )

            self._move_linear_recorded(
                name="descent",
                target_pose_mm_deg=(
                    waypoints["grasp"]
                ),
                velocity_mm_s=(
                    self.descent_velocity
                ),
            )

            # Save open-gripper grasp observation.
            self._force_record_now(
                "before_gripper_close"
            )
            self._set_phase(
                phase="before_gripper_close",
                motion_active=False,
            )

            # Do not record repeated stationary samples during the
            # one-second digital-output pulse.
            self.gripper.command(
                DigitalGripper.CLOSED,
                wait=True,
            )

            # Save exactly one closed-gripper transition observation.
            self._force_record_now(
                "after_gripper_close"
            )
            self._set_phase(
                phase="after_gripper_close",
                motion_active=False,
            )

            self._move_linear_recorded(
                name="lift",
                target_pose_mm_deg=(
                    waypoints["lift"]
                ),
                velocity_mm_s=self.lift_velocity,
            )

            success = True

        except Exception as exc:
            failure_reason = (
                f"{type(exc).__name__}: {exc}"
            )
            self.get_logger().error(
                f"Episode failed: {failure_reason}"
            )
            raise

        finally:
            self._end_recording()

            duration = (
                time.monotonic() - started_at
            )

            metadata = {
                "episode_id": episode_id,
                "instruction": self.instruction,
                "task": self.task,
                "robot": "Doosan A0509",
                "camera": "Stereolabs ZED 2i",
                "camera_mount": (
                    "eye_in_hand_vertical"
                ),
                "coordinate_frame": "base",
                "position_unit": "meter",
                "rotation_unit": "radian",
                "robot_control_position_unit": (
                    "millimeter"
                ),
                "robot_control_rotation_unit": (
                    "degree"
                ),
                "gripper_convention": {
                    "0": "closed",
                    "1": "open",
                },
                "record_frequency_hz": self.frequency,
                "num_steps": self._step_index,
                "duration_sec": duration,
                "success": bool(success),
                "failure_reason": failure_reason,
                "demonstration_type": (
                    "automatic_waypoint_cube_pick"
                ),
                "training_semantics": (
                    "The preparation move to the random "
                    "start pose is not recorded. Motion "
                    "samples are recorded at 10 Hz while "
                    "alignment, descent, and lift are active. "
                    "Stationary gripper pulse frames are "
                    "excluded except for one transition sample."
                ),
                "random_start_pose_mm_deg": (
                    start_pose.astype(float).tolist()
                ),
                "waypoints_mm_deg": {
                    name: pose.astype(float).tolist()
                    for name, pose in waypoints.items()
                },
                "workspace_mm": {
                    "x_min": self.x_min_mm,
                    "x_max": self.x_max_mm,
                    "y_min": self.y_min_mm,
                    "y_max": self.y_max_mm,
                    "start_z": self.start_z_mm,
                    "min_start_xy_distance": (
                        self.min_start_xy_distance_mm
                    ),
                },
                "stationary_filter": {
                    "min_translation_delta_m": (
                        self.min_translation_delta_m
                    ),
                    "min_rotation_delta_rad": (
                        self.min_rotation_delta_rad
                    ),
                    "skipped_stationary_samples": (
                        self._skipped_stationary
                    ),
                },
                "pose_errors": self._pose_errors,
                "image_errors": self._image_errors,
            }

            if self._episode_dir is not None:
                (
                    self._episode_dir
                    / "metadata.json"
                ).write_text(
                    json.dumps(
                        metadata,
                        ensure_ascii=False,
                        indent=2,
                    ),
                    encoding="utf-8",
                )

                if self._step_index >= 2:
                    output_path = process_episode(
                        self._episode_dir
                    )
                    self.get_logger().info(
                        f"Saved actions: {output_path}"
                    )
                else:
                    self.get_logger().warning(
                        "Not enough steps to generate actions"
                    )

            self.get_logger().info(
                f"Saved episode: {episode_id}, "
                f"steps={self._step_index}, "
                f"duration={duration:.2f}s, "
                f"success={success}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = AutomaticCubeDatasetRecorder()
    executor = MultiThreadedExecutor(
        num_threads=4
    )
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    spin_thread.start()

    try:
        node.run_episode()

    except KeyboardInterrupt:
        node.get_logger().warning(
            "Interrupted"
        )

    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
