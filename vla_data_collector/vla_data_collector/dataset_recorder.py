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


class DatasetRecorder(Node):
    """Replay Cartesian waypoints and record ZED RGB, TCP pose, and gripper."""

    def __init__(self) -> None:
        # Matches collector.yaml key: /dsr01/dataset_recorder
        super().__init__("dataset_recorder", namespace="dsr01")

        self.declare_parameter(
            "image_topic", "/zed/zed_node/rgb/color/rect/image"
        )
        self.declare_parameter("waypoint_path", "waypoints/cube_pick.json")
        self.declare_parameter("dataset_root", "raw_dataset")
        self.declare_parameter("episode_id", "episode_000001")
        self.declare_parameter("record_frequency_hz", 10.0)
        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "a0509")
        self.declare_parameter("gripper_output_index", 1)
        self.declare_parameter("gripper_settle_time_sec", 0.6)
        self.declare_parameter("move_velocity_mm_s", 30.0)
        self.declare_parameter("move_acceleration_mm_s2", 60.0)
        self.declare_parameter("waypoint_pause_sec", 0.2)
        self.declare_parameter("jpeg_quality", 95)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.waypoint_path = Path(
            str(self.get_parameter("waypoint_path").value)
        ).expanduser()
        self.dataset_root = Path(
            str(self.get_parameter("dataset_root").value)
        ).expanduser()
        self.episode_id = str(self.get_parameter("episode_id").value)
        self.frequency = float(
            self.get_parameter("record_frequency_hz").value
        )
        self.move_velocity = float(
            self.get_parameter("move_velocity_mm_s").value
        )
        self.move_acceleration = float(
            self.get_parameter("move_acceleration_mm_s2").value
        )
        self.waypoint_pause = float(
            self.get_parameter("waypoint_pause_sec").value
        )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        if self.frequency <= 0:
            raise ValueError("record_frequency_hz must be positive")
        if self.move_velocity <= 0 or self.move_acceleration <= 0:
            raise ValueError("move velocity and acceleration must be positive")
        if self.waypoint_pause < 0:
            raise ValueError("waypoint_pause_sec must be >= 0")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in 1..100")

        self.robot = DoosanInterface(
            node=self,
            robot_id=str(self.get_parameter("robot_id").value),
            robot_model=str(self.get_parameter("robot_model").value),
        )
        self.gripper = DigitalGripper(
            node=self,
            robot=self.robot,
            output_index=int(
                self.get_parameter("gripper_output_index").value
            ),
            settle_time_sec=float(
                self.get_parameter("gripper_settle_time_sec").value
            ),
            initial_state=DigitalGripper.OPEN,
        )

        self.waypoint_data = self._load_waypoints()

        self.episode_dir = self.dataset_root / self.episode_id
        self.image_dir = self.episode_dir / "images"
        self.steps_path = self.episode_dir / "steps.jsonl"
        if self.episode_dir.exists():
            raise FileExistsError(
                f"Episode directory already exists: {self.episode_dir}"
            )
        self.image_dir.mkdir(parents=True, exist_ok=False)

        self.bridge = CvBridge()
        self._image_lock = threading.Lock()
        self._latest_image: Optional[np.ndarray] = None
        self._latest_image_stamp: Optional[float] = None
        self._recording = False
        self._step_index = 0
        self._start_monotonic = 0.0

        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.frequency, self._record_step)

    def _load_waypoints(self) -> dict:
        if not self.waypoint_path.exists():
            raise FileNotFoundError(
                f"Waypoint file not found: {self.waypoint_path}"
            )

        data = json.loads(self.waypoint_path.read_text(encoding="utf-8"))
        waypoints = data.get("waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError("waypoints must be a non-empty list")

        for index, waypoint in enumerate(waypoints):
            pose = np.asarray(waypoint.get("tcp_pose"), dtype=np.float64)
            gripper = waypoint.get("gripper")
            if pose.shape != (6,) or not np.isfinite(pose).all():
                raise ValueError(f"waypoint {index}: invalid tcp_pose")
            if int(gripper) not in (0, 1):
                raise ValueError(f"waypoint {index}: gripper must be 0 or 1")

        return data

    def _image_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            stamp = float(msg.header.stamp.sec) + (
                float(msg.header.stamp.nanosec) * 1e-9
            )
            with self._image_lock:
                self._latest_image = image.copy()
                self._latest_image_stamp = stamp
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")

    def wait_for_first_image(self, timeout_sec: float = 15.0) -> None:
        # The executor is already spinning in another thread.
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self._image_lock:
                if self._latest_image is not None:
                    self.get_logger().info(
                        f"Receiving ZED image: {self.image_topic}"
                    )
                    return
            time.sleep(0.05)
        raise TimeoutError(f"No image received from {self.image_topic}")

    def _record_step(self) -> None:
        if not self._recording:
            return

        with self._image_lock:
            if self._latest_image is None:
                return
            image = self._latest_image.copy()
            image_stamp = self._latest_image_stamp

        try:
            tcp_pose = self.robot.get_tcp_pose().as_array()
        except Exception as exc:
            self.get_logger().error(f"TCP pose read failed: {exc}")
            return

        index = self._step_index
        image_name = f"{index:06d}.jpg"
        image_path = self.image_dir / image_name
        saved = cv2.imwrite(
            str(image_path),
            image,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not saved:
            self.get_logger().error(f"Failed to save image: {image_path}")
            return

        record = {
            "step_index": index,
            "timestamp": time.monotonic() - self._start_monotonic,
            "image_timestamp": image_stamp,
            "image": f"images/{image_name}",
            "tcp_pose": tcp_pose.astype(float).tolist(),
            "gripper": int(self.gripper.state),
        }

        try:
            with self.steps_path.open("a", encoding="utf-8") as file:
                file.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception:
            image_path.unlink(missing_ok=True)
            raise

        self._step_index += 1

    def run_episode(self) -> None:
        self.wait_for_first_image()
        self.robot.set_autonomous_mode()

        waypoints = self.waypoint_data["waypoints"]

        # Put the gripper in the first waypoint's state before robot motion.
        initial_gripper = int(waypoints[0]["gripper"])
        self.gripper.command(initial_gripper, wait=True)

        self._start_monotonic = time.monotonic()
        self._recording = True

        try:
            for waypoint in waypoints:
                name = str(waypoint.get("name", "unnamed"))
                target_pose = waypoint["tcp_pose"]
                requested_gripper = int(waypoint["gripper"])

                self.get_logger().info(f"Moving to waypoint: {name}")
                self.robot.move_linear(
                    target_pose=target_pose,
                    velocity_mm_s=self.move_velocity,
                    acceleration_mm_s2=self.move_acceleration,
                )

                # Waypoint gripper state means state after arriving there.
                if requested_gripper != self.gripper.state:
                    self.gripper.command(requested_gripper, wait=True)

                if self.waypoint_pause > 0:
                    time.sleep(self.waypoint_pause)
        finally:
            self._recording = False

        success = input("큐브 집기 성공? [y/N]: ").strip().lower() == "y"
        metadata = {
            "episode_id": self.episode_id,
            "instruction": self.waypoint_data.get(
                "instruction", "pick up the cube"
            ),
            "robot": "Doosan A0509",
            "camera": "Stereolabs ZED 2i",
            "camera_mount": "eye_in_hand_vertical",
            "coordinate_frame": "base",
            "position_unit": "meter",
            "rotation_unit": "radian",
            "gripper_convention": {"0": "closed", "1": "open"},
            "gripper_output_index": int(
                self.get_parameter("gripper_output_index").value
            ),
            "record_frequency_hz": self.frequency,
            "num_steps": self._step_index,
            "success": success,
        }
        (self.episode_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.get_logger().info(
            f"Saved episode: {self.episode_dir} ({self._step_index} steps)"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetRecorder()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_episode()
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted")
    except Exception as exc:
        node.get_logger().error(f"Episode failed: {exc}")
        raise
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
