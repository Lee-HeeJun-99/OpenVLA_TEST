from __future__ import annotations

import json
import hashlib
import math
from pathlib import Path
import threading
import time
from typing import Optional

import numpy as np
import rclpy
import torch
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray, String
from transformers import (
    AutoConfig,
    AutoImageProcessor,
    AutoModelForVision2Seq,
    AutoProcessor,
)

from .common import ACTION_DIM, array_message, checked_array


class OpenVLAInferenceNode(Node):
    """Runs one OpenVLA inference only after the previous robot command finishes."""

    def __init__(self) -> None:
        super().__init__("openvla_inference")

        self.declare_parameter(
            "model_path",
            "/home/ubuntu/robot_ws/src/openvla/runs/47a0ec7fc4ec123775a391911046cf33cf9ed83f+doosan_a0509+b8+lr-0.0005+lora-r32+dropout-0.0--handguide_10hz_resampled--image_aug",
        )
        self.declare_parameter("unnorm_key", "doosan_a0509")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("use_flash_attention", True)
        self.declare_parameter("default_instruction", "pick up the cube")
        self.declare_parameter("minimum_period_sec", 0.1)
        self.declare_parameter("center_crop_scale", 1.0)
        self.declare_parameter("do_sample", False)
        self.declare_parameter("temperature", 1.0)
        self.declare_parameter("top_p", 1.0)
        self.declare_parameter("log_image_hash", True)
        self.declare_parameter("repeat_action_epsilon", 1.0e-12)
        self.declare_parameter("repeated_action_warn_count", 3)

        self.latest_image: Optional[Image] = None
        self.instruction = str(self.get_parameter("default_instruction").value)
        self.enabled = False
        self.robot_ready = False
        self.inference_busy = False
        self.last_inference_time = 0.0
        self.last_action: Optional[np.ndarray] = None
        self.action_repeat_count = 0
        self.lock = threading.Lock()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.action_publisher = self.create_publisher(
            Float64MultiArray, "/vla/raw_action", 10
        )
        self.status_publisher = self.create_publisher(String, "/vla/model_status", 10)

        self.create_subscription(Image, "/vla/image_rgb", self._image_callback, qos)
        self.create_subscription(String, "/vla/instruction", self._instruction_callback, 10)
        self.create_subscription(Bool, "/vla/enable", self._enable_callback, 10)
        self.create_subscription(Bool, "/vla/robot_ready", self._ready_callback, 10)

        self._load_model()
        self.timer = self.create_timer(0.02, self._tick)

    def _load_model(self) -> None:
        self._register_openvla_auto_classes()

        model_path = str(self.get_parameter("model_path").value)
        device = self._resolve_device(str(self.get_parameter("device").value))
        use_flash = bool(self.get_parameter("use_flash_attention").value)
        torch_dtype = torch.bfloat16 if device.startswith("cuda") else torch.float32

        self.get_logger().info(f"Loading OpenVLA from {model_path}")
        self.processor = AutoProcessor.from_pretrained(
            model_path,
            trust_remote_code=True,
        )

        kwargs = {
            "torch_dtype": torch_dtype,
            "low_cpu_mem_usage": True,
            "trust_remote_code": True,
        }
        if use_flash:
            kwargs["attn_implementation"] = "flash_attention_2"

        try:
            self.model = AutoModelForVision2Seq.from_pretrained(
                model_path,
                **kwargs,
            ).to(device)
        except Exception:
            # Allows deployment without flash-attn.
            kwargs.pop("attn_implementation", None)
            self.get_logger().warning(
                "Flash Attention loading failed; retrying with default attention."
            )
            self.model = AutoModelForVision2Seq.from_pretrained(
                model_path,
                **kwargs,
            ).to(device)

        self.model.eval()
        self.device = device
        self.torch_dtype = torch_dtype
        self._load_dataset_statistics(model_path)
        self.get_logger().info("OpenVLA loaded")

    def _register_openvla_auto_classes(self) -> None:
        try:
            from prismatic.extern.hf.configuration_prismatic import OpenVLAConfig
            from prismatic.extern.hf.modeling_prismatic import (
                OpenVLAForActionPrediction,
            )
            from prismatic.extern.hf.processing_prismatic import (
                PrismaticImageProcessor,
                PrismaticProcessor,
            )
        except Exception as exc:
            self.get_logger().warning(
                "Local OpenVLA registration skipped; relying on trust_remote_code. "
                f"reason={exc}"
            )
            return

        registrations = (
            (AutoConfig.register, ("openvla", OpenVLAConfig)),
            (AutoImageProcessor.register, (OpenVLAConfig, PrismaticImageProcessor)),
            (AutoProcessor.register, (OpenVLAConfig, PrismaticProcessor)),
            (
                AutoModelForVision2Seq.register,
                (OpenVLAConfig, OpenVLAForActionPrediction),
            ),
        )
        for register, args in registrations:
            try:
                register(*args)
            except ValueError:
                pass

    def _resolve_device(self, configured_device: str) -> str:
        device = configured_device.strip().lower()
        if device == "auto":
            return "cuda:0" if torch.cuda.is_available() else "cpu"
        if device.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warning(
                f"Requested {configured_device}, but CUDA is unavailable; using CPU."
            )
            return "cpu"
        return configured_device

    def _load_dataset_statistics(self, model_path: str) -> None:
        statistics_path = Path(model_path).expanduser() / "dataset_statistics.json"
        if not statistics_path.is_file():
            self.get_logger().warning(
                f"No dataset_statistics.json found at {statistics_path}; "
                "predict_action may reject the configured unnorm_key."
            )
            return

        with statistics_path.open("r", encoding="utf-8") as file:
            norm_stats = json.load(file)
        self.model.norm_stats = norm_stats
        self.model.config.norm_stats = norm_stats
        self.get_logger().info(
            "Loaded dataset statistics for: " + ", ".join(sorted(norm_stats.keys()))
        )

    def _image_callback(self, msg: Image) -> None:
        with self.lock:
            self.latest_image = msg

    def _instruction_callback(self, msg: String) -> None:
        self.instruction = msg.data.strip() or self.instruction

    def _enable_callback(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled != self.enabled:
            self._reset_action_tracking()
        self.enabled = enabled

    def _ready_callback(self, msg: Bool) -> None:
        self.robot_ready = bool(msg.data)

    def _tick(self) -> None:
        minimum_period = float(self.get_parameter("minimum_period_sec").value)
        now = time.monotonic()

        if (
            not self.enabled
            or not self.robot_ready
            or self.inference_busy
            or now - self.last_inference_time < minimum_period
        ):
            return

        with self.lock:
            image_msg = self.latest_image
        if image_msg is None:
            return

        self.inference_busy = True
        self.robot_ready = False
        threading.Thread(
            target=self._infer,
            args=(image_msg, self.instruction),
            daemon=True,
        ).start()

    @torch.inference_mode()
    def _infer(self, image_msg: Image, instruction: str) -> None:
        status = String()
        try:
            rgb = self._image_msg_to_rgb_array(image_msg)
            pil_image = PILImage.fromarray(rgb)
            pil_image = self._apply_center_crop(pil_image)

            prompt = (
                f"In: What action should the robot take to {instruction.lower()}?\n"
                "Out:"
            )
            inputs = self.processor(prompt, pil_image)
            inputs = inputs.to(self.device, dtype=self.torch_dtype)
            generation_kwargs = self._generation_kwargs()

            action = self.model.predict_action(
                **inputs,
                unnorm_key=str(self.get_parameter("unnorm_key").value),
                **generation_kwargs,
            )
            action = checked_array(action, ACTION_DIM, "OpenVLA action")
            repeat_count = self._update_action_repeat_count(action)
            self.action_publisher.publish(array_message(action))

            status.data = "inference_ok"
            self.status_publisher.publish(status)
            image_stamp = self._stamp_to_seconds(image_msg)
            image_hash = self._image_hash(pil_image)
            self.get_logger().info(
                "inference_ok:"
                f"action={action.tolist()}, "
                f"image_stamp={image_stamp:.9f}, "
                f"image_hash={image_hash}, "
                f"action_repeat_count={repeat_count}, "
                f"do_sample={generation_kwargs['do_sample']}"
            )
            warn_count = int(self.get_parameter("repeated_action_warn_count").value)
            if warn_count > 0 and repeat_count >= warn_count:
                self.get_logger().warning(
                    "repeated_action_detected:"
                    f"count={repeat_count}, "
                    f"epsilon={float(self.get_parameter('repeat_action_epsilon').value)}, "
                    f"image_hash={image_hash}, "
                    f"action={action.tolist()}"
                )
            self.last_inference_time = time.monotonic()
        except Exception as exc:
            status.data = f"inference_error:{exc}"
            self.status_publisher.publish(status)
            self.get_logger().error(status.data)
        finally:
            self.inference_busy = False

    def _image_msg_to_rgb_array(self, msg: Image) -> np.ndarray:
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
            raise ValueError(
                f"invalid image step {msg.step} for width {msg.width} "
                f"and {channels} channels"
            )

        data = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = int(msg.height) * int(msg.step)
        if data.size < expected_size:
            raise ValueError(
                f"image data too short: got {data.size}, expected {expected_size}"
            )

        rows = data[:expected_size].reshape(int(msg.height), int(msg.step))
        image = rows[:, :row_width].reshape(int(msg.height), int(msg.width), channels)

        if encoding in ("rgb8", "8uc3"):
            rgb = image
        elif encoding == "bgr8":
            rgb = image[:, :, ::-1]
        elif encoding == "rgba8":
            rgb = image[:, :, :3]
        elif encoding == "bgra8":
            rgb = image[:, :, 2::-1]
        else:
            rgb = np.repeat(image, 3, axis=2)

        return np.ascontiguousarray(rgb)

    def _apply_center_crop(self, image: PILImage.Image) -> PILImage.Image:
        crop_scale = float(self.get_parameter("center_crop_scale").value)
        if crop_scale <= 0.0 or crop_scale > 1.0:
            raise ValueError("center_crop_scale must be in the range (0, 1]")
        if crop_scale >= 0.999:
            return image

        width, height = image.size
        crop_ratio = math.sqrt(crop_scale)
        crop_width = max(1, int(round(width * crop_ratio)))
        crop_height = max(1, int(round(height * crop_ratio)))
        left = (width - crop_width) // 2
        top = (height - crop_height) // 2
        cropped = image.crop((left, top, left + crop_width, top + crop_height))
        resampling = getattr(PILImage, "Resampling", PILImage).BICUBIC
        return cropped.resize((width, height), resampling)

    def _generation_kwargs(self) -> dict:
        do_sample = bool(self.get_parameter("do_sample").value)
        kwargs = {"do_sample": do_sample}
        if not do_sample:
            return kwargs

        temperature = float(self.get_parameter("temperature").value)
        top_p = float(self.get_parameter("top_p").value)
        if temperature <= 0.0:
            raise ValueError("temperature must be > 0 when do_sample is true")
        if not 0.0 < top_p <= 1.0:
            raise ValueError("top_p must be in the range (0, 1]")
        kwargs["temperature"] = temperature
        kwargs["top_p"] = top_p
        return kwargs

    def _reset_action_tracking(self) -> None:
        self.last_action = None
        self.action_repeat_count = 0

    def _update_action_repeat_count(self, action: np.ndarray) -> int:
        epsilon = float(self.get_parameter("repeat_action_epsilon").value)
        if (
            self.last_action is not None
            and np.allclose(action, self.last_action, rtol=0.0, atol=epsilon)
        ):
            self.action_repeat_count += 1
        else:
            self.action_repeat_count = 1
        self.last_action = action.copy()
        return self.action_repeat_count

    def _stamp_to_seconds(self, msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9

    def _image_hash(self, image: PILImage.Image) -> str:
        if not bool(self.get_parameter("log_image_hash").value):
            return "disabled"
        rgb = np.asarray(image.convert("RGB"), dtype=np.uint8)
        return hashlib.blake2s(rgb.tobytes(), digest_size=8).hexdigest()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenVLAInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
