from __future__ import annotations

import argparse
import json
import math
import random
import threading
import time
from pathlib import Path
from typing import Optional, Sequence

import rclpy
from dsr_msgs2.srv import GetCurrentPosx, MoveLine, SetRobotMode, SetToolDigitalOutput
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import cv2
    from cv_bridge import CvBridge
except Exception:
    cv2 = None
    CvBridge = None


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image"
DATASET_ROOT = "raw_dataset_test"
EPISODE_COUNT = 1
EPISODE_ID = "auto"
INSTRUCTION = "pick up the cube"
TASK = "cube_pick"
RANDOM_SEED = None

RECORD_FREQUENCY_HZ = 10.0
MAX_IMAGE_AGE_SEC = 0.10
JPEG_QUALITY = 95

BLOCK_MAP = {
    "x": (428.0, 627.0),
    "y": (-217.0, 305.0),
    "z": (300.0, 400.0),
}
BLOCK_RPY = (49.02, 178.24, -127.62)
MAX_RANDOM_ATTEMPTS = 1000

MOVE_VELOCITY_MM_S = 40.0
MOVE_ANGULAR_VELOCITY_DEG_S = 40.0
MOVE_ACCELERATION_MM_S2 = 40.0
MOVE_ANGULAR_ACCELERATION_DEG_S2 = 40.0

BLOCK_ALIGN_XYZ = (539.72, 31.99, 300.0)
BLOCK_GRASP_XYZ = (539.72, 31.99, 224.85)
BLOCK_LIFT_XYZ = BLOCK_ALIGN_XYZ

INITIAL_HOLD_SEC = 0.10
ALIGNMENT_HOLD_SEC = 0.05
GRASP_HOLD_SEC = 0.05
AFTER_GRIPPER_CLOSE_HOLD_SEC = 0.05
FINAL_HOLD_SEC = 0.20

GRIPPER_OPEN_OUTPUT_INDEX = 1
GRIPPER_CLOSE_OUTPUT_INDEX = 2
GRIPPER_PULSE_TIME_SEC = 1.0

CONFIRM_BEFORE_RUN = True


def parse_args(args: Optional[Sequence[str]]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Run randomized cube-pick episodes and save OpenVLA-style data.",
    )
    parser.add_argument("--episodes", type=int, default=EPISODE_COUNT)
    parser.add_argument("--dataset-root", default=DATASET_ROOT)
    parser.add_argument("--episode-id", default=EPISODE_ID)
    parser.add_argument("--image-topic", default=IMAGE_TOPIC)
    parser.add_argument("--record-hz", type=float, default=RECORD_FREQUENCY_HZ)
    parser.add_argument("--seed", type=int, default=RANDOM_SEED)
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="Start the full automatic run without the one-time safety prompt.",
    )
    return parser.parse_known_args(args)


def robot_pose_to_dataset_pose(pose_mm_deg: Sequence[float]) -> list[float]:
    if len(pose_mm_deg) != 6:
        raise ValueError("Robot TCP pose must contain six values")

    pose = [float(value) for value in pose_mm_deg]
    if not all(math.isfinite(value) for value in pose):
        raise ValueError(f"Robot TCP pose contains invalid values: {pose}")

    return [
        pose[0] * 0.001,
        pose[1] * 0.001,
        pose[2] * 0.001,
        math.radians(pose[3]),
        math.radians(pose[4]),
        math.radians(pose[5]),
    ]


def estimate_move_duration_sec(
    start_pose: Sequence[float],
    target_pose: Sequence[float],
) -> float:
    linear_distance_mm = math.sqrt(
        sum((float(target_pose[index]) - float(start_pose[index])) ** 2 for index in range(3))
    )
    angular_distance_deg = max(
        abs(float(target_pose[index]) - float(start_pose[index])) for index in range(3, 6)
    )
    linear_duration = linear_distance_mm / MOVE_VELOCITY_MM_S
    angular_duration = angular_distance_deg / MOVE_ANGULAR_VELOCITY_DEG_S
    return max(linear_duration, angular_duration, 0.1)


def interpolate_pose(
    start_pose: Sequence[float],
    target_pose: Sequence[float],
    ratio: float,
) -> list[float]:
    ratio = min(max(float(ratio), 0.0), 1.0)
    return [
        float(start_pose[index]) + ratio * (float(target_pose[index]) - float(start_pose[index]))
        for index in range(6)
    ]


def wrap_to_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def calculate_action(current: dict, following: dict) -> list[float]:
    current_pose = [float(value) for value in current["tcp_pose"]]
    next_pose = [float(value) for value in following["tcp_pose"]]

    if len(current_pose) != 6 or len(next_pose) != 6:
        raise ValueError("tcp_pose must contain six values")

    return [
        next_pose[0] - current_pose[0],
        next_pose[1] - current_pose[1],
        next_pose[2] - current_pose[2],
        wrap_to_pi(next_pose[3] - current_pose[3]),
        wrap_to_pi(next_pose[4] - current_pose[4]),
        wrap_to_pi(next_pose[5] - current_pose[5]),
        float(following["gripper"]),
    ]


def process_episode(episode_dir: Path) -> Path:
    steps_path = episode_dir / "steps.jsonl"
    if not steps_path.exists():
        raise FileNotFoundError(steps_path)

    steps = [
        json.loads(line)
        for line in steps_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if len(steps) < 2:
        raise ValueError("At least two recorded steps are required")

    for index in range(len(steps) - 1):
        steps[index]["action"] = calculate_action(steps[index], steps[index + 1])

    steps[-1]["action"] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(steps[-1]["gripper"])]

    output_path = episode_dir / "steps_with_actions.jsonl"
    with output_path.open("w", encoding="utf-8") as file:
        for step in steps:
            file.write(json.dumps(step, ensure_ascii=False) + "\n")

    return output_path


class DoosanServices:
    DR_BASE = 0
    DR_MV_MOD_ABS = 0
    DR_MV_RA_DUPLICATE = 0
    ROBOT_MODE_AUTONOMOUS = 1

    def __init__(self, node: rclpy.node.Node, robot_id: str) -> None:
        self.node = node
        service_prefix = f"/{robot_id.strip('/')}"
        self._callback_group = ReentrantCallbackGroup()

        self._get_current_posx = node.create_client(
            GetCurrentPosx,
            f"{service_prefix}/aux_control/get_current_posx",
            callback_group=self._callback_group,
        )
        self._move_line = node.create_client(
            MoveLine,
            f"{service_prefix}/motion/move_line",
            callback_group=self._callback_group,
        )
        self._set_robot_mode = node.create_client(
            SetRobotMode,
            f"{service_prefix}/system/set_robot_mode",
            callback_group=self._callback_group,
        )
        self._set_tool_digital_output = node.create_client(
            SetToolDigitalOutput,
            f"{service_prefix}/io/set_tool_digital_output",
            callback_group=self._callback_group,
        )

    def _call_service(
        self,
        client,
        request,
        service_name: str,
        timeout_sec: float,
        service_wait_sec: float = 5.0,
    ):
        if not client.wait_for_service(timeout_sec=service_wait_sec):
            raise TimeoutError(f"Service not available: {service_name}")

        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                break
            time.sleep(0.005)

        if not future.done():
            raise TimeoutError(f"Service call timed out: {service_name}")

        result = future.result()
        if result is None:
            raise RuntimeError(f"Service call returned no result: {service_name}")
        return result

    def set_autonomous_mode(self) -> None:
        request = SetRobotMode.Request()
        request.robot_mode = self.ROBOT_MODE_AUTONOMOUS
        result = self._call_service(
            self._set_robot_mode,
            request,
            "set_robot_mode",
            timeout_sec=5.0,
        )
        if not result.success:
            raise RuntimeError("set_robot_mode failed")

    def get_tcp_pose(self) -> list[float]:
        request = GetCurrentPosx.Request()
        request.ref = self.DR_BASE
        result = self._call_service(
            self._get_current_posx,
            request,
            "get_current_posx",
            timeout_sec=5.0,
        )

        if not result.success:
            raise RuntimeError("get_current_posx failed")
        if not result.task_pos_info:
            raise RuntimeError("get_current_posx returned no pose data")

        pose = [float(value) for value in result.task_pos_info[0].data[:6]]
        if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
            raise RuntimeError(f"Invalid TCP pose: {pose}")
        return pose

    def move_linear(self, target_pose: Sequence[float], label: str) -> None:
        pose = [float(value) for value in target_pose]
        if len(pose) != 6:
            raise ValueError("target_pose must contain six values")

        request = MoveLine.Request()
        request.pos = pose
        request.vel = [MOVE_VELOCITY_MM_S, MOVE_ANGULAR_VELOCITY_DEG_S]
        request.acc = [MOVE_ACCELERATION_MM_S2, MOVE_ANGULAR_ACCELERATION_DEG_S2]
        request.time = 0.0
        request.radius = 0.0
        request.ref = self.DR_BASE
        request.mode = self.DR_MV_MOD_ABS
        request.blend_type = self.DR_MV_RA_DUPLICATE
        request.sync_type = 0

        self.node.get_logger().info(f"Move {label}: {[round(value, 2) for value in pose]}")
        result = self._call_service(
            self._move_line,
            request,
            "move_line",
            timeout_sec=60.0,
        )
        if not result.success:
            raise RuntimeError(f"move_line failed: {label}, pose={pose}")

    def set_tool_digital_output(self, index: int, value: int) -> None:
        request = SetToolDigitalOutput.Request()
        request.index = int(index)
        request.value = int(value)
        result = self._call_service(
            self._set_tool_digital_output,
            request,
            "set_tool_digital_output",
            timeout_sec=5.0,
        )
        if not result.success:
            raise RuntimeError(f"set_tool_digital_output failed: index={index}, value={value}")


class DigitalGripper:
    CLOSED = 0
    OPEN = 1

    def __init__(
        self,
        robot: DoosanServices,
        open_output_index: int,
        close_output_index: int,
        pulse_time_sec: float,
    ) -> None:
        self.robot = robot
        self.open_output_index = int(open_output_index)
        self.close_output_index = int(close_output_index)
        self.pulse_time_sec = float(pulse_time_sec)
        self.state = self.OPEN

    def _pulse_tool_output(self, index: int, pulse_count: int) -> None:
        for _ in range(pulse_count):
            self.robot.set_tool_digital_output(index, 1)
            time.sleep(self.pulse_time_sec)
            self.robot.set_tool_digital_output(index, 0)
            time.sleep(self.pulse_time_sec)

    def open(self) -> None:
        self.robot.node.get_logger().info("Gripper OPEN")
        self._pulse_tool_output(self.open_output_index, pulse_count=2)
        self.state = self.OPEN

    def close(self) -> None:
        self.robot.node.get_logger().info("Gripper CLOSED")
        self._pulse_tool_output(self.close_output_index, pulse_count=1)
        self.state = self.CLOSED


class EpisodeRecorder:
    def __init__(
        self,
        node: rclpy.node.Node,
        robot: DoosanServices,
        image_topic: str,
        dataset_root: Path,
        frequency_hz: float,
        max_image_age_sec: float,
        jpeg_quality: int,
    ) -> None:
        self.node = node
        self.robot = robot
        self.image_topic = image_topic
        self.dataset_root = dataset_root.expanduser()
        self.frequency_hz = float(frequency_hz)
        self.max_image_age_sec = float(max_image_age_sec)
        self.jpeg_quality = int(jpeg_quality)

        if self.frequency_hz <= 0.0:
            raise ValueError("record frequency must be positive")
        if self.max_image_age_sec <= 0.0:
            raise ValueError("max image age must be positive")

        self._bridge = CvBridge() if CvBridge is not None and cv2 is not None else None

        self._image_lock = threading.Lock()
        self._latest_image_msg: Optional[Image] = None
        self._latest_image_receive_time: Optional[float] = None
        self.node.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self._gripper_lock = threading.Lock()
        self._gripper_state = DigitalGripper.OPEN

        self._planned_pose_lock = threading.Lock()
        self._static_pose_mm_deg: Optional[list[float]] = None
        self._motion_start_pose_mm_deg: Optional[list[float]] = None
        self._motion_target_pose_mm_deg: Optional[list[float]] = None
        self._motion_start_time: Optional[float] = None
        self._motion_start_episode_time = 0.0
        self._motion_duration_sec = 0.0
        self._motion_label = ""
        self.motion_segments: list[dict] = []
        self.static_pose_events: list[dict] = []

        self._record_thread: Optional[threading.Thread] = None
        self._record_stop_event = threading.Event()
        self._record_exception: Optional[BaseException] = None

        self.episode_id = ""
        self.episode_dir: Optional[Path] = None
        self.image_dir: Optional[Path] = None
        self.steps_path: Optional[Path] = None
        self.step_index = 0
        self.episode_start_time = 0.0
        self.last_record_time: Optional[float] = None
        self.last_saved_image_stamp: Optional[float] = None
        self.record_intervals: list[float] = []
        self.skipped_image = 0
        self.pose_errors = 0
        self.image_errors = 0
        self.duplicate_image_count = 0
        self.image_format = ""
        self.planned_pose_samples = 0
        self.feedback_pose_samples = 0

    def _image_callback(self, msg: Image) -> None:
        with self._image_lock:
            self._latest_image_msg = msg
            self._latest_image_receive_time = time.monotonic()

    def wait_for_first_image(self, timeout_sec: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self._image_lock:
                if self._latest_image_msg is not None:
                    self.node.get_logger().info(f"Receiving image: {self.image_topic}")
                    return
            time.sleep(0.05)
        raise TimeoutError(f"No image received from {self.image_topic}")

    def set_gripper_state(self, state: int) -> None:
        with self._gripper_lock:
            self._gripper_state = int(state)

    def get_gripper_state(self) -> int:
        with self._gripper_lock:
            return int(self._gripper_state)

    def set_static_pose(self, pose_mm_deg: Sequence[float]) -> None:
        pose = [float(value) for value in pose_mm_deg]
        if len(pose) != 6:
            raise ValueError("static pose must contain six values")

        with self._planned_pose_lock:
            self._static_pose_mm_deg = pose
            self._motion_start_pose_mm_deg = None
            self._motion_target_pose_mm_deg = None
            self._motion_start_time = None
            self._motion_start_episode_time = 0.0
            self._motion_duration_sec = 0.0
            self._motion_label = ""
            self.static_pose_events.append(
                {
                    "time": self.current_episode_time(),
                    "pose_mm_deg": pose,
                }
            )

    def start_motion_pose(
        self,
        start_pose_mm_deg: Sequence[float],
        target_pose_mm_deg: Sequence[float],
        label: str,
    ) -> float:
        start_pose = [float(value) for value in start_pose_mm_deg]
        target_pose = [float(value) for value in target_pose_mm_deg]
        if len(start_pose) != 6 or len(target_pose) != 6:
            raise ValueError("motion pose must contain six values")

        duration_sec = estimate_move_duration_sec(start_pose, target_pose)
        episode_time = self.current_episode_time()
        with self._planned_pose_lock:
            self._static_pose_mm_deg = target_pose
            self._motion_start_pose_mm_deg = start_pose
            self._motion_target_pose_mm_deg = target_pose
            self._motion_start_time = time.monotonic()
            self._motion_start_episode_time = episode_time
            self._motion_duration_sec = duration_sec
            self._motion_label = label
        return duration_sec

    def finish_motion_pose(self, target_pose_mm_deg: Sequence[float]) -> None:
        target_pose = [float(value) for value in target_pose_mm_deg]
        end_time = self.current_episode_time()

        with self._planned_pose_lock:
            if (
                self._motion_start_pose_mm_deg is not None
                and self._motion_target_pose_mm_deg is not None
                and self._motion_start_time is not None
            ):
                self.motion_segments.append(
                    {
                        "label": self._motion_label,
                        "start_time": self._motion_start_episode_time,
                        "end_time": end_time,
                        "start_pose_mm_deg": list(self._motion_start_pose_mm_deg),
                        "target_pose_mm_deg": list(self._motion_target_pose_mm_deg),
                    }
                )

            self._static_pose_mm_deg = target_pose
            self._motion_start_pose_mm_deg = None
            self._motion_target_pose_mm_deg = None
            self._motion_start_time = None
            self._motion_start_episode_time = 0.0
            self._motion_duration_sec = 0.0
            self._motion_label = ""
            self.static_pose_events.append(
                {
                    "time": end_time,
                    "pose_mm_deg": target_pose,
                }
            )

    def current_planned_pose(self) -> Optional[list[float]]:
        now = time.monotonic()
        with self._planned_pose_lock:
            static_pose = (
                list(self._static_pose_mm_deg)
                if self._static_pose_mm_deg is not None
                else None
            )
            start_pose = (
                list(self._motion_start_pose_mm_deg)
                if self._motion_start_pose_mm_deg is not None
                else None
            )
            target_pose = (
                list(self._motion_target_pose_mm_deg)
                if self._motion_target_pose_mm_deg is not None
                else None
            )
            motion_start_time = self._motion_start_time
            motion_duration_sec = self._motion_duration_sec

        if start_pose is None or target_pose is None or motion_start_time is None:
            return static_pose

        ratio = (now - motion_start_time) / max(motion_duration_sec, 0.001)
        return interpolate_pose(start_pose, target_pose, ratio)

    def current_episode_time(self) -> float:
        if self.episode_start_time <= 0.0:
            return 0.0
        return max(0.0, time.monotonic() - self.episode_start_time)

    def prepare_episode(self, episode_id: str) -> None:
        self.episode_id = episode_id
        self.episode_dir = self.dataset_root / episode_id
        self.image_dir = self.episode_dir / "images"
        self.steps_path = self.episode_dir / "steps.jsonl"

        if self.episode_dir.exists():
            raise FileExistsError(f"Episode directory already exists: {self.episode_dir}")

        self.image_dir.mkdir(parents=True, exist_ok=False)
        self.step_index = 0
        self.episode_start_time = 0.0
        self.last_record_time = None
        self.last_saved_image_stamp = None
        self.record_intervals.clear()
        self.skipped_image = 0
        self.pose_errors = 0
        self.image_errors = 0
        self.duplicate_image_count = 0
        self.image_format = ""
        self.planned_pose_samples = 0
        self.feedback_pose_samples = 0
        self.motion_segments.clear()
        self.static_pose_events.clear()

        with self._planned_pose_lock:
            self._static_pose_mm_deg = None
            self._motion_start_pose_mm_deg = None
            self._motion_target_pose_mm_deg = None
            self._motion_start_time = None
            self._motion_start_episode_time = 0.0
            self._motion_duration_sec = 0.0
            self._motion_label = ""

    def start(self) -> None:
        if self.episode_dir is None or self.image_dir is None or self.steps_path is None:
            raise RuntimeError("Episode directory is not prepared")

        self._record_exception = None
        self._record_stop_event.clear()
        self.episode_start_time = time.monotonic()
        self._record_thread = threading.Thread(
            target=self._record_loop,
            daemon=True,
            name="single_robot_vla_record_loop",
        )
        self._record_thread.start()

    def stop(self) -> None:
        self._record_stop_event.set()

        if self._record_thread is not None:
            self._record_thread.join(timeout=3.0)

        if self._record_exception is not None:
            raise RuntimeError(f"Recording thread failed: {self._record_exception}")

    def finalize_planned_poses(self) -> None:
        if self.steps_path is None or not self.steps_path.exists():
            return

        steps = [
            json.loads(line)
            for line in self.steps_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        if not steps:
            return

        for step in steps:
            pose = self.pose_for_episode_time(float(step["timestamp"]))
            step["tcp_pose"] = robot_pose_to_dataset_pose(pose)
            step["pose_source"] = "planned_actual_duration"

        with self.steps_path.open("w", encoding="utf-8") as file:
            for step in steps:
                file.write(json.dumps(step, ensure_ascii=False) + "\n")

    def pose_for_episode_time(self, timestamp: float) -> list[float]:
        with self._planned_pose_lock:
            motion_segments = [dict(segment) for segment in self.motion_segments]
            static_events = [dict(event) for event in self.static_pose_events]
            static_pose = (
                list(self._static_pose_mm_deg)
                if self._static_pose_mm_deg is not None
                else None
            )

        for segment in motion_segments:
            start_time = float(segment["start_time"])
            end_time = float(segment["end_time"])
            if start_time <= timestamp <= end_time:
                duration = max(end_time - start_time, 0.001)
                ratio = (timestamp - start_time) / duration
                return interpolate_pose(
                    segment["start_pose_mm_deg"],
                    segment["target_pose_mm_deg"],
                    ratio,
                )

        previous_events = [
            event for event in static_events if float(event["time"]) <= timestamp
        ]
        if previous_events:
            return list(previous_events[-1]["pose_mm_deg"])
        if static_events:
            return list(static_events[0]["pose_mm_deg"])
        if static_pose is not None:
            return static_pose

        raise RuntimeError("No planned pose is available for recorded step")

    def _record_loop(self) -> None:
        period = 1.0 / self.frequency_hz
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

    def _latest_image(self) -> tuple[Image, float, float]:
        with self._image_lock:
            if self._latest_image_msg is None or self._latest_image_receive_time is None:
                raise RuntimeError("No image available")
            msg = self._latest_image_msg
            received_time = self._latest_image_receive_time

        image_age = time.monotonic() - received_time
        if image_age > self.max_image_age_sec:
            raise RuntimeError(f"stale image: age={image_age:.3f}s")

        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        return msg, stamp, image_age

    def _record_step(self) -> None:
        if self.image_dir is None or self.steps_path is None:
            raise RuntimeError("Episode paths are not initialized")

        sample_time = time.monotonic()

        try:
            image_msg, image_stamp, image_age = self._latest_image()
        except Exception as exc:
            self.skipped_image += 1
            self.node.get_logger().warning(f"Skipping sample: {exc}")
            return

        pose_source = "planned"
        pose_age = 0.0
        planned_pose = self.current_planned_pose()

        if planned_pose is not None:
            tcp_pose = robot_pose_to_dataset_pose(planned_pose)
            self.planned_pose_samples += 1
        else:
            try:
                pose_request_time = time.monotonic()
                robot_pose = self.robot.get_tcp_pose()
                tcp_pose = robot_pose_to_dataset_pose(robot_pose)
                pose_age = time.monotonic() - pose_request_time
                pose_source = "feedback"
                self.feedback_pose_samples += 1
            except Exception as exc:
                self.pose_errors += 1
                self.node.get_logger().error(f"TCP pose read failed: {exc}")
                return

        if self.last_saved_image_stamp is not None and image_stamp == self.last_saved_image_stamp:
            self.duplicate_image_count += 1

        index = self.step_index
        try:
            image_name = self._save_image(image_msg, index)
        except Exception as exc:
            self.image_errors += 1
            self.node.get_logger().error(f"Image save failed: {exc}")
            return

        now = time.monotonic()
        if self.last_record_time is not None:
            self.record_intervals.append(now - self.last_record_time)
        self.last_record_time = now

        record = {
            "step_index": index,
            "timestamp": now - self.episode_start_time,
            "source_timestamp": sample_time,
            "image_timestamp": image_stamp,
            "image_age_sec": image_age,
            "pose_age_sec": pose_age,
            "pose_source": pose_source,
            "image": f"images/{image_name}",
            "tcp_pose": tcp_pose,
            "gripper": self.get_gripper_state(),
        }

        try:
            with self.steps_path.open("a", encoding="utf-8") as file:
                file.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception:
            (self.image_dir / image_name).unlink(missing_ok=True)
            raise

        self.last_saved_image_stamp = image_stamp
        self.step_index += 1

    def _save_image(self, msg: Image, index: int) -> str:
        if self._bridge is not None and cv2 is not None:
            try:
                image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                image_name = f"{index:06d}.jpg"
                image_path = self.image_dir / image_name
                saved = cv2.imwrite(
                    str(image_path),
                    image,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if saved:
                    self.image_format = "jpg"
                    return image_name
            except Exception as exc:
                self.node.get_logger().warning(f"JPEG save failed, using PPM fallback: {exc}")

        image_name = f"{index:06d}.ppm"
        image_path = self.image_dir / image_name
        image_path.write_bytes(image_msg_to_ppm(msg))
        self.image_format = "ppm"
        return image_name

    def write_metadata(
        self,
        success: bool,
        start_pose: Sequence[float],
        align_pose: Sequence[float],
        grasp_pose: Sequence[float],
        lift_pose: Sequence[float],
        elapsed_sec: float,
        failure_reason: str = "",
    ) -> None:
        if self.episode_dir is None:
            return

        effective_hz = 0.0
        if self.record_intervals:
            mean_interval = sum(self.record_intervals) / len(self.record_intervals)
            if mean_interval > 0.0:
                effective_hz = 1.0 / mean_interval

        metadata = {
            "episode_id": self.episode_id,
            "instruction": INSTRUCTION,
            "task": TASK,
            "robot": f"Doosan {ROBOT_MODEL.upper()}",
            "camera": "Stereolabs ZED",
            "camera_mount": "eye_in_hand_vertical",
            "coordinate_frame": "base",
            "pose_source": "planned_commanded_pose",
            "position_unit": "meter",
            "rotation_unit": "radian",
            "robot_control_position_unit": "millimeter",
            "robot_control_rotation_unit": "degree",
            "image_topic": self.image_topic,
            "image_format": self.image_format,
            "record_frequency_hz": self.frequency_hz,
            "effective_record_frequency_hz": effective_hz,
            "num_steps": self.step_index,
            "duration_sec": elapsed_sec,
            "success": bool(success),
            "failure_reason": failure_reason,
            "gripper_convention": {"0": "closed", "1": "open"},
            "gripper_value_semantics": "commanded_state",
            "demonstration_type": "automatic_random_start_cube_pick",
            "training_semantics": (
                "The move to the random start pose is not recorded. "
                "Recorded samples begin at the random start pose and contain "
                "alignment, descent, grasp-close, and lift."
            ),
            "random_start_pose_mm_deg": [float(value) for value in start_pose],
            "route_poses_mm_deg": {
                "alignment": [float(value) for value in align_pose],
                "grasp": [float(value) for value in grasp_pose],
                "lift": [float(value) for value in lift_pose],
            },
            "motion_segments": self.motion_segments,
            "workspace_mm": {
                "x": list(BLOCK_MAP["x"]),
                "y": list(BLOCK_MAP["y"]),
                "z": list(BLOCK_MAP["z"]),
            },
            "move_velocity_mm_s": MOVE_VELOCITY_MM_S,
            "move_angular_velocity_deg_s": MOVE_ANGULAR_VELOCITY_DEG_S,
            "move_acceleration_mm_s2": MOVE_ACCELERATION_MM_S2,
            "move_angular_acceleration_deg_s2": MOVE_ANGULAR_ACCELERATION_DEG_S2,
            "skipped_image": self.skipped_image,
            "pose_errors": self.pose_errors,
            "image_errors": self.image_errors,
            "duplicate_image_count": self.duplicate_image_count,
            "planned_pose_samples": self.planned_pose_samples,
            "feedback_pose_samples": self.feedback_pose_samples,
        }

        (self.episode_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


def image_msg_to_ppm(msg: Image) -> bytes:
    encoding = msg.encoding.lower()
    width = int(msg.width)
    height = int(msg.height)

    if encoding in ("rgb8",):
        rgb = packed_image_rows(msg, channels=3)
    elif encoding in ("bgr8", "8uc3"):
        rgb = bgr_to_rgb(packed_image_rows(msg, channels=3))
    elif encoding == "rgba8":
        rgb = rgba_to_rgb(packed_image_rows(msg, channels=4))
    elif encoding in ("bgra8", "8uc4"):
        rgb = bgra_to_rgb(packed_image_rows(msg, channels=4))
    elif encoding in ("mono8", "8uc1"):
        rgb = mono_to_rgb(packed_image_rows(msg, channels=1))
    else:
        raise ValueError(f"Unsupported image encoding for PPM fallback: {msg.encoding}")

    header = f"P6\n{width} {height}\n255\n".encode("ascii")
    return header + rgb


def packed_image_rows(msg: Image, channels: int) -> bytes:
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    row_width = width * channels
    raw = bytes(msg.data)

    if step < row_width:
        raise ValueError(f"Invalid image step: step={step}, row_width={row_width}")
    if len(raw) < height * step:
        raise ValueError(f"Image data is shorter than expected: {len(raw)} < {height * step}")

    if step == row_width:
        return raw[: height * row_width]

    packed = bytearray(height * row_width)
    for row in range(height):
        src_start = row * step
        dst_start = row * row_width
        packed[dst_start : dst_start + row_width] = raw[src_start : src_start + row_width]
    return bytes(packed)


def bgr_to_rgb(data: bytes) -> bytes:
    output = bytearray(len(data))
    for index in range(0, len(data), 3):
        output[index : index + 3] = bytes((data[index + 2], data[index + 1], data[index]))
    return bytes(output)


def rgba_to_rgb(data: bytes) -> bytes:
    output = bytearray((len(data) // 4) * 3)
    for src in range(0, len(data), 4):
        dst = (src // 4) * 3
        output[dst : dst + 3] = data[src : src + 3]
    return bytes(output)


def bgra_to_rgb(data: bytes) -> bytes:
    output = bytearray((len(data) // 4) * 3)
    for src in range(0, len(data), 4):
        dst = (src // 4) * 3
        output[dst : dst + 3] = bytes((data[src + 2], data[src + 1], data[src]))
    return bytes(output)


def mono_to_rgb(data: bytes) -> bytes:
    output = bytearray(len(data) * 3)
    for src, value in enumerate(data):
        dst = src * 3
        output[dst : dst + 3] = bytes((value, value, value))
    return bytes(output)


class RandomStartSampler:
    def __init__(self) -> None:
        self.used_positions: set[tuple[float, float, float]] = set()

    def sample(self) -> list[float]:
        for _ in range(MAX_RANDOM_ATTEMPTS):
            position = (
                round(random.uniform(*BLOCK_MAP["x"]), 2),
                round(random.uniform(*BLOCK_MAP["y"]), 2),
                round(random.uniform(*BLOCK_MAP["z"]), 2),
            )

            if position in self.used_positions:
                continue

            self.used_positions.add(position)
            return [*position, *BLOCK_RPY]

        raise RuntimeError("Could not sample a new unique random start pose")


class AutoEpisodeRunner:
    def __init__(
        self,
        node: rclpy.node.Node,
        robot: DoosanServices,
        gripper: DigitalGripper,
        recorder: EpisodeRecorder,
        episode_id: str,
        episode_count: int,
    ) -> None:
        self.node = node
        self.robot = robot
        self.gripper = gripper
        self.recorder = recorder
        self.configured_episode_id = episode_id
        self.episode_count = int(episode_count)
        self.sampler = RandomStartSampler()

        if self.episode_count <= 0:
            raise ValueError("episode count must be positive")

    def run(self) -> None:
        self.recorder.wait_for_first_image()
        self.robot.set_autonomous_mode()
        self.robot.get_tcp_pose()

        for episode_number in range(1, self.episode_count + 1):
            episode_id = self._resolve_episode_id(episode_number)
            self.run_episode(episode_id, episode_number)

    def run_episode(self, episode_id: str, episode_number: int) -> None:
        start_pose = self.sampler.sample()
        align_pose = [*BLOCK_ALIGN_XYZ, *BLOCK_RPY]
        grasp_pose = [*BLOCK_GRASP_XYZ, *BLOCK_RPY]
        lift_pose = [*BLOCK_LIFT_XYZ, *BLOCK_RPY]

        self.recorder.prepare_episode(episode_id)

        self.node.get_logger().info(
            f"Episode {episode_number}/{self.episode_count}: {episode_id}, "
            f"random_start={[round(value, 2) for value in start_pose]}"
        )

        success = False
        failure_reason = ""
        recording_started = False
        started_at = time.monotonic()

        try:
            self.robot.move_linear(start_pose, "random_start_unrecorded")

            self.gripper.open()
            self.recorder.set_gripper_state(DigitalGripper.OPEN)
            self.recorder.set_static_pose(start_pose)

            self.recorder.start()
            recording_started = True

            time.sleep(INITIAL_HOLD_SEC)
            self._recorded_move(start_pose, align_pose, "alignment")
            time.sleep(ALIGNMENT_HOLD_SEC)

            self._recorded_move(align_pose, grasp_pose, "descent_to_grasp")
            time.sleep(GRASP_HOLD_SEC)

            self.recorder.set_gripper_state(DigitalGripper.CLOSED)
            self.gripper.close()
            time.sleep(AFTER_GRIPPER_CLOSE_HOLD_SEC)

            self._recorded_move(grasp_pose, lift_pose, "lift")
            time.sleep(FINAL_HOLD_SEC)

            success = True

        except Exception as exc:
            failure_reason = f"{type(exc).__name__}: {exc}"
            self.node.get_logger().error(f"Episode failed: {failure_reason}")
            raise

        finally:
            if recording_started:
                try:
                    self.recorder.stop()
                except Exception as exc:
                    success = False
                    if not failure_reason:
                        failure_reason = f"{type(exc).__name__}: {exc}"
                    self.node.get_logger().error(f"Recording stop failed: {failure_reason}")

            try:
                self.recorder.finalize_planned_poses()
            except Exception as exc:
                success = False
                if not failure_reason:
                    failure_reason = f"{type(exc).__name__}: {exc}"
                self.node.get_logger().error(f"Planned pose finalization failed: {failure_reason}")

            elapsed_sec = time.monotonic() - started_at
            self.recorder.write_metadata(
                success=success,
                start_pose=start_pose,
                align_pose=align_pose,
                grasp_pose=grasp_pose,
                lift_pose=lift_pose,
                elapsed_sec=elapsed_sec,
                failure_reason=failure_reason,
            )

            if self.recorder.step_index >= 2 and self.recorder.episode_dir is not None:
                actions_path = process_episode(self.recorder.episode_dir)
                self.node.get_logger().info(f"Saved actions: {actions_path}")
            else:
                self.node.get_logger().warning("Not enough steps to generate actions")

            self.node.get_logger().info(
                f"Saved {episode_id}: steps={self.recorder.step_index}, success={success}"
            )

    def _recorded_move(
        self,
        start_pose: Sequence[float],
        target_pose: Sequence[float],
        label: str,
    ) -> None:
        duration_sec = self.recorder.start_motion_pose(start_pose, target_pose, label)
        self.node.get_logger().info(
            f"Recording segment {label}: planned_duration={duration_sec:.2f}s"
        )

        try:
            self.robot.move_linear(target_pose, label)
        finally:
            self.recorder.finish_motion_pose(target_pose)

    def _resolve_episode_id(self, episode_number: int) -> str:
        configured = self.configured_episode_id.strip()
        if configured and configured.lower() != "auto":
            if self.episode_count == 1:
                return configured
            return f"{configured}_{episode_number:06d}"

        self.recorder.dataset_root.mkdir(parents=True, exist_ok=True)
        used_indices = []
        for path in self.recorder.dataset_root.glob("episode_*"):
            suffix = path.name.removeprefix("episode_")
            if suffix.isdigit():
                used_indices.append(int(suffix))
        return f"episode_{max(used_indices, default=0) + 1:06d}"


def main(args: Optional[Sequence[str]] = None) -> None:
    cli_args, ros_args = parse_args(args)

    if cli_args.seed is not None:
        random.seed(cli_args.seed)

    if CONFIRM_BEFORE_RUN and not cli_args.no_confirm:
        input(
            f"{cli_args.episodes} episode(s) will run automatically. "
            "Check workspace and emergency stop, then press Enter: "
        )

    rclpy.init(args=ros_args)
    node = rclpy.create_node("single_robot_simple_py", namespace=ROBOT_ID)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        robot = DoosanServices(node=node, robot_id=ROBOT_ID)
        gripper = DigitalGripper(
            robot=robot,
            open_output_index=GRIPPER_OPEN_OUTPUT_INDEX,
            close_output_index=GRIPPER_CLOSE_OUTPUT_INDEX,
            pulse_time_sec=GRIPPER_PULSE_TIME_SEC,
        )
        recorder = EpisodeRecorder(
            node=node,
            robot=robot,
            image_topic=cli_args.image_topic,
            dataset_root=Path(cli_args.dataset_root),
            frequency_hz=cli_args.record_hz,
            max_image_age_sec=MAX_IMAGE_AGE_SEC,
            jpeg_quality=JPEG_QUALITY,
        )
        runner = AutoEpisodeRunner(
            node=node,
            robot=robot,
            gripper=gripper,
            recorder=recorder,
            episode_id=cli_args.episode_id,
            episode_count=cli_args.episodes,
        )
        runner.run()

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
