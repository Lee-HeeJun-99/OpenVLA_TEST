from __future__ import annotations

import json
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from .doosan_interface import DoosanInterface
from .gripper_interface import DigitalGripper
from .make_actions import process_episode


class TerminalKeyReader:
    def __init__(self) -> None:
        self._fd: Optional[int] = None
        self._old_settings = None

    def __enter__(self) -> "TerminalKeyReader":
        if sys.stdin.isatty():
            self._fd = sys.stdin.fileno()
            self._old_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def read_key(self) -> Optional[str]:
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        key = sys.stdin.read(1)
        return key.lower() if key else None


class HandguideRecorder(Node):
    """Record fixed-rate hand-guided demonstrations for OpenVLA."""

    ROBOT_POSITION_TO_DATASET_SCALE = 0.001
    ROBOT_ROTATION_TO_DATASET_SCALE = np.pi / 180.0

    def __init__(self) -> None:
        super().__init__("handguide_recorder", namespace="dsr01")

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("pose_source", "service")
        self.declare_parameter("pose_topic", "/doosan/current_pose")
        self.declare_parameter("dataset_root", "raw_dataset")
        self.declare_parameter("episode_id", "auto")
        self.declare_parameter("instruction", "pick up the cube")
        self.declare_parameter("task", "cube_pick")

        self.declare_parameter("record_frequency_hz", 10.0)
        self.declare_parameter("max_image_age_sec", 0.08)
        self.declare_parameter("max_pose_age_sec", 0.05)
        self.declare_parameter("record_only_when_changed", False)
        self.declare_parameter("min_position_delta_m", 0.001)
        self.declare_parameter("min_rotation_delta_rad", 0.005)

        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("generate_actions_on_stop", True)
        self.declare_parameter("final_hold_samples", 3)

        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "a0509")
        self.declare_parameter("initial_gripper_state", 1)
        self.declare_parameter("gripper_open_output_index", 1)
        self.declare_parameter("gripper_close_output_index", 2)
        self.declare_parameter("gripper_pulse_time_sec", 1.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.pose_source = str(self.get_parameter("pose_source").value).strip().lower()
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.dataset_root = Path(str(self.get_parameter("dataset_root").value)).expanduser()
        self.episode_id = self._resolve_episode_id(str(self.get_parameter("episode_id").value))

        self.frequency = float(self.get_parameter("record_frequency_hz").value)
        self.max_image_age = float(self.get_parameter("max_image_age_sec").value)
        self.max_pose_age = float(self.get_parameter("max_pose_age_sec").value)
        self.record_only_when_changed = bool(self.get_parameter("record_only_when_changed").value)
        self.min_position_delta_m = float(self.get_parameter("min_position_delta_m").value)
        self.min_rotation_delta_rad = float(self.get_parameter("min_rotation_delta_rad").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.generate_actions_on_stop = bool(self.get_parameter("generate_actions_on_stop").value)
        self.final_hold_samples = int(self.get_parameter("final_hold_samples").value)

        self._validate_parameters()

        self.robot = DoosanInterface(
            node=self,
            robot_id=str(self.get_parameter("robot_id").value),
            robot_model=str(self.get_parameter("robot_model").value),
        )
        self.gripper = DigitalGripper(
            node=self,
            robot=self.robot,
            open_output_index=int(self.get_parameter("gripper_open_output_index").value),
            close_output_index=int(self.get_parameter("gripper_close_output_index").value),
            pulse_time_sec=float(self.get_parameter("gripper_pulse_time_sec").value),
            initial_state=int(self.get_parameter("initial_gripper_state").value),
        )

        self.episode_dir = self.dataset_root / self.episode_id
        self.image_dir = self.episode_dir / "images"
        self.steps_path = self.episode_dir / "steps.jsonl"
        if self.episode_dir.exists():
            raise FileExistsError(f"Episode directory already exists: {self.episode_dir}")
        self.image_dir.mkdir(parents=True, exist_ok=False)

        self._image_lock = threading.Lock()
        self._latest_image_msg: Optional[Image] = None
        self._latest_image_receive_monotonic: Optional[float] = None

        self._pose_lock = threading.Lock()
        self._latest_topic_pose: Optional[np.ndarray] = None
        self._latest_topic_pose_receive_monotonic: Optional[float] = None

        self._gripper_lock = threading.Lock()
        self._recorded_gripper_state = int(self.get_parameter("initial_gripper_state").value)
        self._gripper_command_busy = False

        self._recording = False
        self._step_index = 0
        self._start_monotonic = 0.0
        self._last_recorded_pose: Optional[np.ndarray] = None
        self._last_recorded_gripper_state: Optional[int] = None
        self._last_record_monotonic: Optional[float] = None
        self._last_saved_image_stamp: Optional[float] = None

        self._record_intervals: list[float] = []
        self._skipped_stale_image = 0
        self._skipped_stale_pose = 0
        self._skipped_small_motion = 0
        self._pose_errors = 0
        self._image_errors = 0
        self._duplicate_image_stamp_count = 0
        self._missed_schedule_count = 0

        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        if self.pose_source == "topic":
            self.create_subscription(
                Float64MultiArray,
                self.pose_topic,
                self._pose_callback,
                50,
            )

    def _validate_parameters(self) -> None:
        if self.pose_source not in ("service", "topic"):
            raise ValueError("pose_source must be 'service' or 'topic'")
        if self.frequency <= 0.0:
            raise ValueError("record_frequency_hz must be positive")
        if self.max_image_age <= 0.0:
            raise ValueError("max_image_age_sec must be positive")
        if self.max_pose_age <= 0.0:
            raise ValueError("max_pose_age_sec must be positive")
        if self.min_position_delta_m < 0.0:
            raise ValueError("min_position_delta_m must be >= 0")
        if self.min_rotation_delta_rad < 0.0:
            raise ValueError("min_rotation_delta_rad must be >= 0")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in 1..100")
        if self.final_hold_samples < 0:
            raise ValueError("final_hold_samples must be >= 0")

    def _resolve_episode_id(self, configured_episode_id: str) -> str:
        episode_id = configured_episode_id.strip()
        if episode_id and episode_id.lower() != "auto":
            return episode_id

        self.dataset_root.mkdir(parents=True, exist_ok=True)
        used_indices = []
        for path in self.dataset_root.glob("episode_*"):
            suffix = path.name.removeprefix("episode_")
            if suffix.isdigit():
                used_indices.append(int(suffix))
        return f"episode_{max(used_indices, default=0) + 1:06d}"

    @classmethod
    def _convert_robot_pose_to_dataset_pose(cls, pose: np.ndarray) -> np.ndarray:
        converted = np.asarray(pose, dtype=np.float64).copy()
        if converted.shape != (6,):
            raise ValueError("TCP pose must have six values")
        if not np.isfinite(converted).all():
            raise ValueError("TCP pose contains a non-finite value")
        converted[:3] *= cls.ROBOT_POSITION_TO_DATASET_SCALE
        converted[3:] *= cls.ROBOT_ROTATION_TO_DATASET_SCALE
        return converted

    def _image_callback(self, msg: Image) -> None:
        with self._image_lock:
            self._latest_image_msg = msg
            self._latest_image_receive_monotonic = time.monotonic()

    def _pose_callback(self, msg: Float64MultiArray) -> None:
        try:
            pose = np.asarray(msg.data, dtype=np.float64)
            if pose.shape != (6,) or not np.isfinite(pose).all():
                raise ValueError(f"invalid pose shape/value: {pose}")
            with self._pose_lock:
                self._latest_topic_pose = pose.copy()
                self._latest_topic_pose_receive_monotonic = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"Pose topic conversion failed: {exc}")

    def wait_for_first_image(self, timeout_sec: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self._image_lock:
                if self._latest_image_msg is not None:
                    self.get_logger().info(f"Receiving image: {self.image_topic}")
                    return
            time.sleep(0.05)
        raise TimeoutError(f"No image received from {self.image_topic}")

    def wait_for_first_pose(self, timeout_sec: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                self._read_robot_pose()
                self.get_logger().info(f"Receiving pose via {self.pose_source}")
                return
            except Exception:
                time.sleep(0.05)
        raise TimeoutError(f"No robot pose received via {self.pose_source}")

    def _read_robot_pose(self) -> tuple[np.ndarray, float]:
        if self.pose_source == "service":
            request_started = time.monotonic()
            pose = self.robot.get_tcp_pose().as_array()
            return np.asarray(pose, dtype=np.float64), time.monotonic() - request_started

        with self._pose_lock:
            if self._latest_topic_pose is None or self._latest_topic_pose_receive_monotonic is None:
                raise RuntimeError(f"No pose received on {self.pose_topic}")
            pose = self._latest_topic_pose.copy()
            received_monotonic = self._latest_topic_pose_receive_monotonic

        pose_age = time.monotonic() - received_monotonic
        if pose_age > self.max_pose_age:
            self._skipped_stale_pose += 1
            raise RuntimeError(f"stale pose: age={pose_age:.3f}s")
        return pose, pose_age

    def _latest_image(self) -> tuple[np.ndarray, float, float]:
        with self._image_lock:
            if self._latest_image_msg is None or self._latest_image_receive_monotonic is None:
                raise RuntimeError("No image available")
            msg = self._latest_image_msg
            received_monotonic = self._latest_image_receive_monotonic

        image_age = time.monotonic() - received_monotonic
        if image_age > self.max_image_age:
            raise RuntimeError(f"stale image: age={image_age:.3f}s")

        image = self._image_msg_to_bgr_array(msg)
        image_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        return image, image_stamp, image_age

    @staticmethod
    def _image_msg_to_bgr_array(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        channel_counts = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
            "8uc1": 1,
            "8uc3": 3,
            "8uc4": 4,
        }
        if encoding not in channel_counts:
            raise ValueError(f"unsupported image encoding: {msg.encoding}")

        channels = channel_counts[encoding]
        row_width = int(msg.width) * channels
        if msg.step < row_width:
            raise ValueError(f"invalid image step {msg.step} for width {msg.width} and {channels} channels")

        data = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = int(msg.height) * int(msg.step)
        if data.size < expected_size:
            raise ValueError(f"image data too short: got {data.size}, expected {expected_size}")

        rows = data[:expected_size].reshape(int(msg.height), int(msg.step))
        image = rows[:, :row_width].reshape(int(msg.height), int(msg.width), channels)

        if encoding in ("bgr8", "8uc3"):
            bgr = image
        elif encoding == "rgb8":
            bgr = image[:, :, ::-1]
        elif encoding == "bgra8":
            bgr = image[:, :, :3]
        elif encoding == "rgba8":
            bgr = image[:, :, 2::-1]
        else:
            bgr = np.repeat(image, 3, axis=2)
        return np.ascontiguousarray(bgr)

    def _current_gripper_state(self) -> int:
        with self._gripper_lock:
            return self._recorded_gripper_state

    def _set_recorded_gripper_state(self, state: int) -> bool:
        state = int(state)
        if state not in (DigitalGripper.CLOSED, DigitalGripper.OPEN):
            raise ValueError("gripper state must be 0 or 1")

        with self._gripper_lock:
            if self._gripper_command_busy:
                self.get_logger().warning("Ignoring gripper command: command already running")
                return False
            self._recorded_gripper_state = state
            self._gripper_command_busy = True

        threading.Thread(target=self._command_gripper, args=(state,), daemon=True).start()
        return True

    def _command_gripper(self, state: int) -> None:
        try:
            self.gripper.command(state, wait=True)
        except Exception as exc:
            self.get_logger().error(f"Gripper command failed: {exc}")
        finally:
            with self._gripper_lock:
                self._gripper_command_busy = False

    def _should_record(self, tcp_pose: np.ndarray, gripper_state: int, force: bool = False) -> bool:
        if force or self._last_recorded_pose is None:
            return True
        if self._last_recorded_gripper_state != gripper_state:
            return True
        if not self.record_only_when_changed:
            return True

        delta = tcp_pose - self._last_recorded_pose
        if float(np.linalg.norm(delta[:3])) >= self.min_position_delta_m:
            return True
        if float(np.linalg.norm(delta[3:])) >= self.min_rotation_delta_rad:
            return True

        self._skipped_small_motion += 1
        return False

    def _record_step(self, force: bool = False) -> bool:
        sample_monotonic = time.monotonic()

        try:
            image, image_stamp, image_age = self._latest_image()
        except Exception as exc:
            self._skipped_stale_image += 1
            self.get_logger().warning(f"Skipping sample: {exc}")
            return False

        try:
            robot_tcp_pose, pose_age = self._read_robot_pose()
            tcp_pose = self._convert_robot_pose_to_dataset_pose(robot_tcp_pose)
        except Exception as exc:
            self._pose_errors += 1
            self.get_logger().error(f"TCP pose read failed: {exc}")
            return False

        gripper_state = self._current_gripper_state()
        if not self._should_record(tcp_pose, gripper_state, force=force):
            return False

        if self._last_saved_image_stamp is not None and image_stamp == self._last_saved_image_stamp:
            self._duplicate_image_stamp_count += 1

        index = self._step_index
        image_name = f"{index:06d}.jpg"
        image_path = self.image_dir / image_name
        saved = cv2.imwrite(str(image_path), image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not saved:
            self._image_errors += 1
            self.get_logger().error(f"Failed to save image: {image_path}")
            return False

        now = time.monotonic()
        if self._last_record_monotonic is not None:
            self._record_intervals.append(now - self._last_record_monotonic)
        self._last_record_monotonic = now

        record = {
            "step_index": index,
            "timestamp": sample_monotonic - self._start_monotonic,
            "source_timestamp": sample_monotonic,
            "image_timestamp": image_stamp,
            "image_age_sec": image_age,
            "pose_age_sec": pose_age,
            "image": f"images/{image_name}",
            "tcp_pose": tcp_pose.astype(float).tolist(),
            "gripper": int(gripper_state),
        }

        try:
            with self.steps_path.open("a", encoding="utf-8") as file:
                file.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception:
            image_path.unlink(missing_ok=True)
            raise

        self._last_recorded_pose = tcp_pose.copy()
        self._last_recorded_gripper_state = gripper_state
        self._last_saved_image_stamp = image_stamp
        self._step_index += 1

        if self._step_index % 10 == 0:
            self.get_logger().info(
                f"recorded:{self._step_index} gripper={gripper_state} "
                f"image_age={image_age:.3f}s pose_age={pose_age:.3f}s"
            )
        return True

    def _print_controls(self) -> None:
        print(
            "\n=== Hand-guided OpenVLA recorder ===\n"
            "Move continuously and avoid long pauses.\n"
            "Recommended: XY center -> vertical descent -> close -> lift\n\n"
            "Controls:\n"
            "  o : open gripper\n"
            "  c : close gripper\n"
            "  s : stop and mark success\n"
            "  f : stop and mark failure\n"
            "  q : abort and mark failure\n",
            flush=True,
        )

    def _record_final_hold(self, period: float) -> None:
        for _ in range(self.final_hold_samples):
            if not rclpy.ok():
                return
            time.sleep(period)
            self._record_step(force=True)

    def run_episode(self) -> None:
        self.wait_for_first_image()
        self.wait_for_first_pose()
        self._print_controls()
        input("Press Enter to start recording...")

        self._start_monotonic = time.monotonic()
        self._recording = True
        success = False
        stopped = False
        period = 1.0 / self.frequency
        next_sample = time.monotonic()

        self.get_logger().info(
            f"Started recording: {self.episode_dir}, sample_hz={self.frequency}, "
            f"pose_source={self.pose_source}, fixed_rate={not self.record_only_when_changed}"
        )

        with TerminalKeyReader() as keys:
            while rclpy.ok() and not stopped:
                key = keys.read_key()
                if key == "o":
                    if self._set_recorded_gripper_state(DigitalGripper.OPEN):
                        self.get_logger().info("operator_gripper:open")
                elif key == "c":
                    if self._set_recorded_gripper_state(DigitalGripper.CLOSED):
                        self.get_logger().info("operator_gripper:close")
                elif key == "s":
                    success = True
                    stopped = True
                elif key in ("f", "q"):
                    success = False
                    stopped = True

                now = time.monotonic()
                if now >= next_sample:
                    self._record_step()
                    next_sample += period
                    if next_sample < now:
                        self._missed_schedule_count += 1
                        next_sample = now + period
                time.sleep(0.002)

        self._record_final_hold(period)
        self._recording = False
        self._write_metadata(success=success)

        if self.generate_actions_on_stop and self._step_index >= 2:
            output_path = process_episode(self.episode_dir)
            self.get_logger().info(f"Saved actions: {output_path}")
        elif self._step_index < 2:
            self.get_logger().warning("Not enough recorded steps to generate actions")

        self.get_logger().info(
            f"Saved episode: {self.episode_dir} ({self._step_index} steps, success={success})"
        )

    def _write_metadata(self, success: bool) -> None:
        effective_hz = 0.0
        if self._record_intervals:
            mean_interval = float(np.mean(self._record_intervals))
            if mean_interval > 0.0:
                effective_hz = 1.0 / mean_interval

        metadata = {
            "episode_id": self.episode_id,
            "instruction": str(self.get_parameter("instruction").value),
            "task": str(self.get_parameter("task").value),
            "robot": "Doosan A0509",
            "camera": "Stereolabs ZED 2i",
            "camera_mount": "eye_in_hand_vertical",
            "coordinate_frame": "base",
            "position_unit": "meter",
            "rotation_unit": "radian",
            "robot_control_position_unit": "millimeter",
            "robot_control_rotation_unit": "degree",
            "pose_source": self.pose_source,
            "pose_topic": self.pose_topic if self.pose_source == "topic" else "",
            "image_topic": self.image_topic,
            "record_frequency_hz": self.frequency,
            "effective_record_frequency_hz": effective_hz,
            "record_only_when_changed": self.record_only_when_changed,
            "min_position_delta_m": self.min_position_delta_m,
            "min_rotation_delta_rad": self.min_rotation_delta_rad,
            "max_image_age_sec": self.max_image_age,
            "max_pose_age_sec": self.max_pose_age,
            "final_hold_samples": self.final_hold_samples,
            "num_steps": self._step_index,
            "success": bool(success),
            "gripper_convention": {"0": "closed", "1": "open"},
            "gripper_value_semantics": "commanded_state",
            "gripper_open_output_index": int(self.get_parameter("gripper_open_output_index").value),
            "gripper_close_output_index": int(self.get_parameter("gripper_close_output_index").value),
            "gripper_pulse_time_sec": float(self.get_parameter("gripper_pulse_time_sec").value),
            "skipped_stale_image": self._skipped_stale_image,
            "skipped_stale_pose": self._skipped_stale_pose,
            "skipped_small_motion": self._skipped_small_motion,
            "pose_errors": self._pose_errors,
            "image_errors": self._image_errors,
            "duplicate_image_stamp_count": self._duplicate_image_stamp_count,
            "missed_schedule_count": self._missed_schedule_count,
        }
        (self.episode_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandguideRecorder()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_episode()
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted")
        node._write_metadata(success=False)
    except Exception as exc:
        node.get_logger().error(f"Hand-guided episode failed: {exc}")
        node._write_metadata(success=False)
        raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
