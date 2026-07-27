\
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray, String
from transformers import AutoModelForVision2Seq, AutoProcessor

from .common import ACTION_DIM, array_message, checked_array


class OpenVLAInferenceNode(Node):
    """Runs one OpenVLA inference only after the previous robot command finishes."""

    def __init__(self) -> None:
        super().__init__("openvla_inference")

        self.declare_parameter("model_path", "/path/to/finetuned-openvla")
        self.declare_parameter("unnorm_key", "doosan_cube_pick")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("use_flash_attention", True)
        self.declare_parameter("default_instruction", "pick up the cube")
        self.declare_parameter("minimum_period_sec", 0.1)

        self.bridge = CvBridge()
        self.latest_image: Optional[Image] = None
        self.instruction = str(self.get_parameter("default_instruction").value)
        self.enabled = False
        self.robot_ready = False
        self.inference_busy = False
        self.last_inference_time = 0.0
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
        model_path = str(self.get_parameter("model_path").value)
        device = str(self.get_parameter("device").value)
        use_flash = bool(self.get_parameter("use_flash_attention").value)

        self.get_logger().info(f"Loading OpenVLA from {model_path}")
        self.processor = AutoProcessor.from_pretrained(
            model_path,
            trust_remote_code=True,
        )

        kwargs = {
            "torch_dtype": torch.bfloat16,
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
        self.get_logger().info("OpenVLA loaded")

    def _image_callback(self, msg: Image) -> None:
        with self.lock:
            self.latest_image = msg

    def _instruction_callback(self, msg: String) -> None:
        self.instruction = msg.data.strip() or self.instruction

    def _enable_callback(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

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
            rgb = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
            pil_image = PILImage.fromarray(rgb)

            prompt = (
                f"In: What action should the robot take to {instruction.lower()}?\n"
                "Out:"
            )
            inputs = self.processor(prompt, pil_image)
            inputs = inputs.to(self.device, dtype=torch.bfloat16)

            action = self.model.predict_action(
                **inputs,
                unnorm_key=str(self.get_parameter("unnorm_key").value),
                do_sample=False,
            )
            action = checked_array(action, ACTION_DIM, "OpenVLA action")
            self.action_publisher.publish(array_message(action))

            status.data = "inference_ok"
            self.status_publisher.publish(status)
            self.last_inference_time = time.monotonic()
        except Exception as exc:
            status.data = f"inference_error:{exc}"
            self.status_publisher.publish(status)
            self.get_logger().error(status.data)
        finally:
            self.inference_busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenVLAInferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
