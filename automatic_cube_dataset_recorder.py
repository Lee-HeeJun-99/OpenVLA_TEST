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
    Automatic OpenVLA data collector for a fixed cube.

    Unrecorded preparation:
      1. Open gripper.
      2. Move to a random start pose:
         x=random[x_min, x_max], y=random[y_min, y_max], z=start_z.

    Recorded trajectory:
      random start -> alignment pose -> grasp pose -> close -> lift pose

    Saved convention:
      pose   = [x, y, z, rx, ry, rz] in meter/radian
      action = [dx, dy, dz, dRx, dRy, dRz, gripper]
      gripper: 0=closed, 1=open
    """

    def __init__(self) -> None:
        super().__init__("automatic_cube_dataset_recorder", namespace="dsr01")

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("dataset_root", "raw_dataset")
        self.declare_parameter("episode_id", "auto")
        self.declare_parameter("instruction", "pick up the cube")
        self.declare_parameter("task", "cube_pick")
        self.declare_parameter("record_frequency_hz", 10.0)
        self.declare_parameter("max_image_age_sec", 0.10)
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("seed", 2026)

        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "a0509")

        self.declare_parameter("x_min_mm", 428.25)
        self.declare_parameter("x_max_mm", 627.16)
        self.declare_parameter("y_min_mm", -218.94)
        self.declare_parameter("y_max_mm", 305.11)
        self.declare_parameter("start_z_mm", 400.0)
        self.declare_parameter("min_start_xy_distance_mm", 120.0)
        self.declare_parameter("max_sampling_attempts", 100)

        self.declare_parameter(
            "grasp_pose_mm_deg",
            [539.72, 31.99, 224.85, 46.59, 178.82, -131.11],
        )
        self.declare_parameter("alignment_z_mm", 320.0)
        self.declare_parameter("lift_z_mm", 350.0)

        self.declare_parameter("alignment_duration_sec", 2.8)
        self.declare_parameter("descent_duration_sec", 1.0)
        self.declare_parameter("lift_duration_sec", 1.2)
        self.declare_parameter("minimum_velocity_mm_s", 30.0)
        self.declare_parameter("maximum_velocity_mm_s", 150.0)
        self.declare_parameter("move_acceleration_mm_s2", 200.0)

        self.declare_parameter("initial_hold_sec", 0.10)
        self.declare_parameter("alignment_hold_sec", 0.05)
        self.declare_parameter("grasp_hold_sec", 0.05)
        self.declare_parameter("final_hold_sec", 0.20)

        self.declare_parameter("gripper_open_output_index", 1)
        self.declare_parameter("gripper_close_output_index", 2)
        self.declare_parameter("gripper_pulse_time_sec", 1.0)
        self.declare_parameter("initial_gripper_state", 1)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.dataset_root = Path(str(self.get_parameter("dataset_root").value)).expanduser()
        self.configured_episode_id = str(self.get_parameter("episode_id").value)
        self.instruction = str(self.get_parameter("instruction").value)
        self.task = str(self.get_parameter("task").value)
        self.frequency = float(self.get_parameter("record_frequency_hz").value)
        self.max_image_age = float(self.get_parameter("max_image_age_sec").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))

        self.x_min_mm = float(self.get_parameter("x_min_mm").value)
        self.x_max_mm = float(self.get_parameter("x_max_mm").value)
        self.y_min_mm = float(self.get_parameter("y_min_mm").value)
        self.y_max_mm = float(self.get_parameter("y_max_mm").value)
        self.start_z_mm = float(self.get_parameter("start_z_mm").value)
        self.min_start_xy_distance_mm = float(
            self.get_parameter("min_start_xy_distance_mm").value
        )
        self.max_sampling_attempts = int(
            self.get_parameter("max_sampling_attempts").value
        )

        self.grasp_pose_mm_deg = np.asarray(
            self.get_parameter("grasp_pose_mm_deg").value,
            dtype=np.float64,
        )
        self.alignment_z_mm = float(self.get_parameter("alignment_z_mm").value)
        self.lift_z_mm = float(self.get_parameter("lift_z_mm").value)

        self.alignment_duration_sec = float(
            self.get_parameter("alignment_duration_sec").value
        )
        self.descent_duration_sec = float(
            self.get_parameter("descent_duration_sec").value
        )
        self.lift_duration_sec = float(
            self.get_parameter("lift_duration_sec").value
        )
        self.minimum_velocity_mm_s = float(
            self.get_parameter("minimum_velocity_mm_s").value
        )
        self.maximum_velocity_mm_s = float(
            self.get_parameter("maximum_velocity_mm_s").value
        )
        self.move_acceleration_mm_s2 = float(
            self.get_parameter("move_acceleration_mm_s2").value
        )

        self.initial_hold_sec = float(self.get_parameter("initial_hold_sec").value)
        self.alignment_hold_sec = float(
            self.get_parameter("alignment_hold_sec").value
        )
        self.grasp_hold_sec = float(self.get_parameter("grasp_hold_sec").value)
        self.final_hold_sec = float(self.get_parameter("final_hold_sec").value)

        self._validate_parameters()

        self.robot = DoosanInterface(
            node=self,
            robot_id=str(self.get_parameter("robot_id").value),
            robot_model=str(self.get_parameter("robot_model").value),
        )
        self.gripper = DigitalGripper(
            node=self,
            robot=self.robot,
            open_output_index=int(
                self.get_parameter("gripper_open_output_index").value
            ),
            close_output_index=int(
                self.get_parameter("gripper_close_output_index").value
            ),
            pulse_time_sec=float(
                self.get_parameter("gripper_pulse_time_sec").value
            ),
            initial_state=int(self.get_parameter("initial_gripper_state").value),
        )

        self.bridge = CvBridge()
        self._image_lock = threading.Lock()
        self._latest_image: Optional[np.ndarray] = None
        self._latest_image_stamp: Optional[float] = None
        self._latest_image_receive_time: Optional[float] = None

        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self._record_thread: Optional[threading.Thread] = None
        self._record_exception: Optional[BaseException] = None
        self._record_stop_event = threading.Event()

        self._episode_dir: Optional[Path] = None
        self._image_dir: Optional[Path] = None
        self._steps_path: Optional[Path] = None
        self._episode_start_time = 0.0
        self._step_index = 0
        self._gripper_state = DigitalGripper.OPEN
        self._record_intervals: list[float] = []
        self._last_record_time: Optional[float] = None
        self._skipped_image = 0
        self._pose_errors = 0
        self._image_errors = 0
        self._duplicate_image_count = 0
        self._last_saved_image_stamp: Optional[float] = None

    def _validate_parameters(self) -> None:
        if self.frequency <= 0.0:
            raise ValueError("record_frequency_hz must be positive")
        if self.max_image_age <= 0.0:
            raise ValueError("max_image_age_sec must be positive")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in 1..100")
        if self.x_min_mm >= self.x_max_mm:
            raise ValueError("x_min_mm must be smaller than x_max_mm")
        if self.y_min_mm >= self.y_max_mm:
            raise ValueError("y_min_mm must be smaller than y_max_mm")
        if self.start_z_mm > 400.0:
            raise ValueError("start_z_mm must not exceed 400 mm")
        if self.grasp_pose_mm_deg.shape != (6,):
            raise ValueError("grasp_pose_mm_deg must contain six values")
        if self.alignment_z_mm <= self.grasp_pose_mm_deg[2]:
            raise ValueError("alignment_z_mm must be above grasp z")
        if self.lift_z_mm <= self.grasp_pose_mm_deg[2]:
            raise ValueError("lift_z_mm must be above grasp z")
        if self.minimum_velocity_mm_s <= 0.0:
            raise ValueError("minimum_velocity_mm_s must be positive")
        if self.maximum_velocity_mm_s < self.minimum_velocity_mm_s:
            raise ValueError(
                "maximum_velocity_mm_s must be >= minimum_velocity_mm_s"
            )
        if self.move_acceleration_mm_s2 <= 0.0:
            raise ValueError("move_acceleration_mm_s2 must be positive")

    def _image_callback(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            stamp = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1.0e-9
            )
            with self._image_lock:
                self._latest_image = np.ascontiguousarray(bgr)
                self._latest_image_stamp = stamp
                self._latest_image_receive_time = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")

    def wait_for_first_image(self, timeout_sec: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self._image_lock:
                if self._latest_image is not None:
                    self.get_logger().info(f"Receiving image: {self.image_topic}")
                    return
            time.sleep(0.05)
        raise TimeoutError(f"No image received from {self.image_topic}")

    @staticmethod
    def _mm_deg_to_m_rad(pose: np.ndarray) -> np.ndarray:
        converted = np.asarray(pose, dtype=np.float64).copy()
        converted[:3] *= 0.001
        converted[3:] = np.deg2rad(converted[3:])
        return converted

    def _sample_start_pose_mm_deg(self) -> np.ndarray:
        cube_xy = self.grasp_pose_mm_deg[:2]

        for _ in range(self.max_sampling_attempts):
            x = float(self.rng.uniform(self.x_min_mm, self.x_max_mm))
            y = float(self.rng.uniform(self.y_min_mm, self.y_max_mm))

            if (
                np.linalg.norm(np.asarray([x, y]) - cube_xy)
                >= self.min_start_xy_distance_mm
            ):
                pose = self.grasp_pose_mm_deg.copy()
                pose[0] = x
                pose[1] = y
                pose[2] = self.start_z_mm
                return pose

        raise RuntimeError(
            "Could not sample a valid start point. "
            "Reduce min_start_xy_distance_mm or widen the workspace."
        )

    def _build_poses(
        self,
        start_pose_mm_deg: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        alignment = self.grasp_pose_mm_deg.copy()
        alignment[2] = self.alignment_z_mm

        grasp = self.grasp_pose_mm_deg.copy()

        lift = self.grasp_pose_mm_deg.copy()
        lift[2] = self.lift_z_mm

        return (
            self._mm_deg_to_m_rad(start_pose_mm_deg),
            self._mm_deg_to_m_rad(alignment),
            self._mm_deg_to_m_rad(grasp),
            self._mm_deg_to_m_rad(lift),
        )

    def _velocity_for_segment(
        self,
        start_pose_m_rad: np.ndarray,
        target_pose_m_rad: np.ndarray,
        duration_sec: float,
    ) -> float:
        distance_mm = float(
            np.linalg.norm(target_pose_m_rad[:3] - start_pose_m_rad[:3])
            * 1000.0
        )
        velocity = distance_mm / duration_sec
        return float(
            np.clip(
                velocity,
                self.minimum_velocity_mm_s,
                self.maximum_velocity_mm_s,
            )
        )

    def _resolve_episode_id(self) -> str:
        configured = self.configured_episode_id.strip()
        if configured and configured.lower() != "auto":
            return configured

        self.dataset_root.mkdir(parents=True, exist_ok=True)
        used_indices: list[int] = []

        for path in self.dataset_root.glob("episode_*"):
            suffix = path.name.removeprefix("episode_")
            if suffix.isdigit():
                used_indices.append(int(suffix))

        return f"episode_{max(used_indices, default=0) + 1:06d}"

    def _prepare_episode_directory(self) -> str:
        episode_id = self._resolve_episode_id()
        self._episode_dir = self.dataset_root / episode_id
        self._image_dir = self._episode_dir / "images"
        self._steps_path = self._episode_dir / "steps.jsonl"

        if self._episode_dir.exists():
            raise FileExistsError(
                f"Episode directory already exists: {self._episode_dir}"
            )

        self._image_dir.mkdir(parents=True, exist_ok=False)
        return episode_id

    def _start_recording(self) -> None:
        self._record_stop_event.clear()
        self._record_exception = None
        self._episode_start_time = time.monotonic()
        self._step_index = 0
        self._record_intervals.clear()
        self._last_record_time = None
        self._skipped_image = 0
        self._pose_errors = 0
        self._image_errors = 0
        self._duplicate_image_count = 0
        self._last_saved_image_stamp = None

        self._record_thread = threading.Thread(
            target=self._record_loop,
            daemon=True,
            name="automatic_dataset_record_loop",
        )
        self._record_thread.start()

    def _stop_recording(self) -> None:
        self._record_stop_event.set()

        if self._record_thread is not None:
            self._record_thread.join(timeout=3.0)

        if self._record_exception is not None:
            raise RuntimeError(
                f"Recording thread failed: {self._record_exception}"
            ) from self._record_exception

    def _record_loop(self) -> None:
        period = 1.0 / self.frequency
        next_sample = time.monotonic()

        try:
            while rclpy.ok() and not self._record_stop_event.is_set():
                now = time.monotonic()

                if now >= next_sample:
                    self._record_step()
                    next_sample += period

                    if next_sample < now:
                        next_sample = now + period

                time.sleep(0.001)

        except BaseException as exc:
            self._record_exception = exc
            self._record_stop_event.set()

    def _get_latest_image(self) -> tuple[np.ndarray, float, float]:
        with self._image_lock:
            if (
                self._latest_image is None
                or self._latest_image_stamp is None
                or self._latest_image_receive_time is None
            ):
                raise RuntimeError("No image available")

            image = self._latest_image.copy()
            stamp = float(self._latest_image_stamp)
            received_time = float(self._latest_image_receive_time)

        image_age = time.monotonic() - received_time

        if image_age > self.max_image_age:
            raise RuntimeError(f"stale image: age={image_age:.3f}s")

        return image, stamp, image_age

    def _record_step(self) -> None:
        if self._image_dir is None or self._steps_path is None:
            raise RuntimeError("Episode paths are not initialized")

        try:
            image, image_stamp, image_age = self._get_latest_image()
        except Exception as exc:
            self._skipped_image += 1
            self.get_logger().warning(
                f"Skipping sample: {exc}",
                throttle_duration_sec=1.0,
            )
            return

        try:
            tcp_pose = self.robot.get_tcp_pose().as_array().astype(np.float64)
        except Exception as exc:
            self._pose_errors += 1
            self.get_logger().error(
                f"TCP pose read failed: {exc}",
                throttle_duration_sec=1.0,
            )
            return

        if (
            self._last_saved_image_stamp is not None
            and image_stamp == self._last_saved_image_stamp
        ):
            self._duplicate_image_count += 1

        index = self._step_index
        image_name = f"{index:06d}.jpg"
        image_path = self._image_dir / image_name

        if not cv2.imwrite(
            str(image_path),
            image,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        ):
            self._image_errors += 1
            return

        now = time.monotonic()

        if self._last_record_time is not None:
            self._record_intervals.append(now - self._last_record_time)
        self._last_record_time = now

        record = {
            "step_index": index,
            "timestamp": now - self._episode_start_time,
            "image_timestamp": image_stamp,
            "image_age_sec": image_age,
            "image": f"images/{image_name}",
            "tcp_pose": tcp_pose.astype(float).tolist(),
            "gripper": int(self._gripper_state),
        }

        try:
            with self._steps_path.open("a", encoding="utf-8") as file:
                file.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception:
            image_path.unlink(missing_ok=True)
            raise

        self._last_saved_image_stamp = image_stamp
        self._step_index += 1

    def _move(
        self,
        start_pose: np.ndarray,
        target_pose: np.ndarray,
        duration_sec: float,
        label: str,
    ) -> None:
        velocity = self._velocity_for_segment(
            start_pose,
            target_pose,
            duration_sec,
        )

        self.get_logger().info(
            f"{label} | velocity={velocity:.1f} mm/s | "
            f"target={np.round(target_pose, 5).tolist()}"
        )

        self.robot.move_linear(
            target_pose=target_pose,
            velocity_mm_s=velocity,
            acceleration_mm_s2=self.move_acceleration_mm_s2,
        )

    def _write_metadata(
        self,
        episode_id: str,
        success: bool,
        start_pose_mm_deg: np.ndarray,
        alignment_pose: np.ndarray,
        grasp_pose: np.ndarray,
        lift_pose: np.ndarray,
        elapsed_sec: float,
        failure_reason: str = "",
    ) -> None:
        if self._episode_dir is None:
            return

        effective_hz = 0.0

        if self._record_intervals:
            mean_interval = float(np.mean(self._record_intervals))
            if mean_interval > 0.0:
                effective_hz = 1.0 / mean_interval

        metadata = {
            "episode_id": episode_id,
            "instruction": self.instruction,
            "task": self.task,
            "robot": "Doosan A0509",
            "camera": "Stereolabs ZED 2i",
            "camera_mount": "eye_in_hand_vertical",
            "coordinate_frame": "base",
            "position_unit": "meter",
            "rotation_unit": "radian",
            "robot_control_position_unit": "millimeter",
            "robot_control_rotation_unit": "degree",
            "record_frequency_hz": self.frequency,
            "effective_record_frequency_hz": effective_hz,
            "num_steps": self._step_index,
            "duration_sec": elapsed_sec,
            "success": bool(success),
            "failure_reason": failure_reason,
            "gripper_convention": {"0": "closed", "1": "open"},
            "demonstration_type": "automatic_waypoint_cube_pick",
            "training_semantics": (
                "The move to the random start pose is not recorded. "
                "Samples begin at the random start pose and contain "
                "alignment, descent, grasp, and lift."
            ),
            "random_start_pose_mm_deg": start_pose_mm_deg.astype(float).tolist(),
            "target_poses_m_rad": {
                "alignment": alignment_pose.astype(float).tolist(),
                "grasp": grasp_pose.astype(float).tolist(),
                "lift": lift_pose.astype(float).tolist(),
            },
            "workspace_mm": {
                "x_min": self.x_min_mm,
                "x_max": self.x_max_mm,
                "y_min": self.y_min_mm,
                "y_max": self.y_max_mm,
                "start_z": self.start_z_mm,
                "min_start_xy_distance": self.min_start_xy_distance_mm,
            },
            "skipped_image": self._skipped_image,
            "pose_errors": self._pose_errors,
            "image_errors": self._image_errors,
            "duplicate_image_count": self._duplicate_image_count,
        }

        (self._episode_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def run_episode(self) -> None:
        self.wait_for_first_image()

        episode_id = self._prepare_episode_directory()
        start_pose_mm_deg = self._sample_start_pose_mm_deg()
        start_pose, alignment_pose, grasp_pose, lift_pose = self._build_poses(
            start_pose_mm_deg
        )

        print()
        print(f"Episode: {episode_id}")
        print(
            "Random start [mm, deg]: "
            f"{np.round(start_pose_mm_deg, 2).tolist()}"
        )
        print(
            "Recorded route: random start -> alignment -> "
            "grasp -> close -> lift"
        )
        input(
            "Check the workspace and emergency stop, "
            "then press Enter to start: "
        )

        success = False
        failure_reason = ""
        recording_started = False
        started_at = time.monotonic()

        try:
            # Preparation movement is not recorded.
            self._gripper_state = DigitalGripper.OPEN
            self.gripper.command(DigitalGripper.OPEN, wait=True)

            current_pose = self.robot.get_tcp_pose().as_array()
            preparation_velocity = self._velocity_for_segment(
                current_pose,
                start_pose,
                duration_sec=3.0,
            )

            self.get_logger().info(
                "Moving to random start pose without recording"
            )
            self.robot.move_linear(
                target_pose=start_pose,
                velocity_mm_s=preparation_velocity,
                acceleration_mm_s2=self.move_acceleration_mm_s2,
            )

            self._start_recording()
            recording_started = True

            if self.initial_hold_sec > 0.0:
                time.sleep(self.initial_hold_sec)

            self._move(
                start_pose,
                alignment_pose,
                self.alignment_duration_sec,
                "alignment",
            )

            if self.alignment_hold_sec > 0.0:
                time.sleep(self.alignment_hold_sec)

            self._move(
                alignment_pose,
                grasp_pose,
                self.descent_duration_sec,
                "descent",
            )

            if self.grasp_hold_sec > 0.0:
                time.sleep(self.grasp_hold_sec)

            self._gripper_state = DigitalGripper.CLOSED
            self.gripper.command(DigitalGripper.CLOSED, wait=True)

            self._move(
                grasp_pose,
                lift_pose,
                self.lift_duration_sec,
                "lift",
            )

            if self.final_hold_sec > 0.0:
                time.sleep(self.final_hold_sec)

            success = True

        except Exception as exc:
            failure_reason = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(
                f"Automatic episode failed: {failure_reason}"
            )
            raise

        finally:
            if recording_started:
                self._stop_recording()

            elapsed_sec = time.monotonic() - started_at

            self._write_metadata(
                episode_id=episode_id,
                success=success,
                start_pose_mm_deg=start_pose_mm_deg,
                alignment_pose=alignment_pose,
                grasp_pose=grasp_pose,
                lift_pose=lift_pose,
                elapsed_sec=elapsed_sec,
                failure_reason=failure_reason,
            )

            if self._step_index >= 2 and self._episode_dir is not None:
                process_episode(self._episode_dir)
            else:
                self.get_logger().warning(
                    "Not enough steps to generate actions"
                )

            self.get_logger().info(
                f"Saved {episode_id} | steps={self._step_index} | "
                f"duration={elapsed_sec:.2f}s | success={success}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutomaticCubeDatasetRecorder()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    spin_thread.start()

    try:
        node.run_episode()

    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted")

    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
