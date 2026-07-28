from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import torch
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
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
    """
    Runs OpenVLA inference once per completed robot-control cycle.

    Control cycle:
      1. /vla/enable=True
      2. /vla/robot_ready=True
      3. A new image is received
      4. OpenVLA inference runs
      5. /vla/raw_action is published
      6. Wait for robot_ready=False
      7. Wait for robot_ready=True
      8. Run the next inference using a new image
    """

    def __init__(self) -> None:
        super().__init__("openvla_inference")

        # ------------------------------------------------------------------
        # Model parameters
        # ------------------------------------------------------------------
        self.declare_parameter(
            "model_path",
            (
                "/home/ubuntu/robot_ws/src/openvla/runs/"
                "47a0ec7fc4ec123775a391911046cf33cf9ed83f"
                "+doosan_a0509+b8+lr-0.0005+lora-r32"
                "+dropout-0.0--handguide_10hz_resampled--image_aug"
            ),
        )

        self.declare_parameter(
            "unnorm_key",
            "doosan_a0509",
        )

        self.declare_parameter(
            "device",
            "cuda:0",
        )

        self.declare_parameter(
            "use_flash_attention",
            True,
        )

        self.declare_parameter(
            "default_instruction",
            "pick up the cube",
        )

        # Minimum interval between inference attempts, including failed ones.
        self.declare_parameter(
            "minimum_period_sec",
            0.1,
        )

        # Fraction of image area retained before resizing to original size.
        self.declare_parameter(
            "center_crop_scale",
            1.0,
        )

        self.declare_parameter(
            "do_sample",
            False,
        )

        self.declare_parameter(
            "temperature",
            1.0,
        )

        self.declare_parameter(
            "top_p",
            1.0,
        )

        # ------------------------------------------------------------------
        # Diagnostic parameters
        # ------------------------------------------------------------------
        self.declare_parameter(
            "log_image_hash",
            True,
        )

        self.declare_parameter(
            "log_image_statistics",
            True,
        )

        self.declare_parameter(
            "repeat_action_epsilon",
            1.0e-5,
        )

        self.declare_parameter(
            "repeated_action_warn_count",
            3,
        )

        # ------------------------------------------------------------------
        # Shared state
        # ------------------------------------------------------------------
        self.state_lock = threading.Lock()

        self.latest_image: Optional[Image] = None
        self.latest_image_id = 0
        self.last_used_image_id = 0

        self.instruction = str(
            self.get_parameter(
                "default_instruction"
            ).value
        )

        self.enabled = False
        self.robot_ready = False
        self.inference_busy = False

        # True after an action is published and until the robot completes
        # ready -> not ready -> ready.
        self.command_outstanding = False
        self.robot_became_busy = False

        self.last_inference_attempt_time = 0.0

        self.last_action: Optional[np.ndarray] = None
        self.action_repeat_count = 0

        # Incremented whenever enable state changes.
        # Results produced by an old episode are discarded.
        self.episode_generation = 0

        # ------------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------------
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        default_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        instruction_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.action_publisher = self.create_publisher(
            Float64MultiArray,
            "/vla/raw_action",
            default_qos,
        )

        self.action_valid_publisher = self.create_publisher(
            Bool,
            "/vla/action_valid",
            default_qos,
        )

        self.status_publisher = self.create_publisher(
            String,
            "/vla/model_status",
            default_qos,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            Image,
            "/vla/image_rgb",
            self._image_callback,
            image_qos,
        )

        self.create_subscription(
            String,
            "/vla/instruction",
            self._instruction_callback,
            instruction_qos,
        )

        self.create_subscription(
            Bool,
            "/vla/enable",
            self._enable_callback,
            default_qos,
        )

        self.create_subscription(
            Bool,
            "/vla/robot_ready",
            self._ready_callback,
            default_qos,
        )

        # ------------------------------------------------------------------
        # Model loading
        # ------------------------------------------------------------------
        self._load_model()

        # 50 Hz state-check timer. Actual inference rate is separately limited.
        self.timer = self.create_timer(
            0.02,
            self._tick,
        )

        self._log_parameters()

    # ======================================================================
    # Status helpers
    # ======================================================================

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = str(text)
        self.status_publisher.publish(message)

    def _publish_action_valid(self, valid: bool) -> None:
        message = Bool()
        message.data = bool(valid)
        self.action_valid_publisher.publish(message)

    # ======================================================================
    # Model loading
    # ======================================================================

    def _load_model(self) -> None:
        self._register_openvla_auto_classes()

        model_path = Path(
            str(
                self.get_parameter(
                    "model_path"
                ).value
            )
        ).expanduser()

        self._validate_checkpoint_files(model_path)

        configured_device = str(
            self.get_parameter("device").value
        )

        device = self._resolve_device(
            configured_device
        )

        use_flash_attention = bool(
            self.get_parameter(
                "use_flash_attention"
            ).value
        )

        torch_dtype = (
            torch.bfloat16
            if device.startswith("cuda")
            else torch.float32
        )

        self.get_logger().info(
            f"Loading OpenVLA from {model_path}"
        )

        self.processor = AutoProcessor.from_pretrained(
            str(model_path),
            trust_remote_code=True,
        )

        model_kwargs = {
            "torch_dtype": torch_dtype,
            "low_cpu_mem_usage": True,
            "trust_remote_code": True,
        }

        if use_flash_attention:
            model_kwargs[
                "attn_implementation"
            ] = "flash_attention_2"

        try:
            self.model = (
                AutoModelForVision2Seq.from_pretrained(
                    str(model_path),
                    **model_kwargs,
                ).to(device)
            )

        except Exception as flash_exception:
            if (
                "attn_implementation"
                not in model_kwargs
            ):
                raise

            model_kwargs.pop(
                "attn_implementation",
                None,
            )

            self.get_logger().warning(
                "Flash Attention loading failed; "
                "retrying with default attention. "
                f"reason={flash_exception}"
            )

            self.model = (
                AutoModelForVision2Seq.from_pretrained(
                    str(model_path),
                    **model_kwargs,
                ).to(device)
            )

        self.model.eval()

        self.device = device
        self.torch_dtype = torch_dtype

        self._load_and_validate_dataset_statistics(
            model_path
        )

        self.get_logger().info(
            "OpenVLA loaded:"
            f"device={self.device}, "
            f"dtype={self.torch_dtype}, "
            f"unnorm_key="
            f"{self.get_parameter('unnorm_key').value}"
        )

    def _validate_checkpoint_files(
        self,
        model_path: Path,
    ) -> None:
        if not model_path.is_dir():
            raise FileNotFoundError(
                f"model_path is not a directory: "
                f"{model_path}"
            )

        required_files = (
            "config.json",
            "model.safetensors",
            "processor_config.json",
            "dataset_statistics.json",
        )

        missing_files = [
            filename
            for filename in required_files
            if not (
                model_path / filename
            ).is_file()
        ]

        if missing_files:
            raise FileNotFoundError(
                "OpenVLA checkpoint is incomplete:"
                f"path={model_path}, "
                f"missing={missing_files}"
            )

        self.get_logger().info(
            "Checkpoint files verified:"
            + ",".join(required_files)
        )

    def _register_openvla_auto_classes(
        self,
    ) -> None:
        try:
            from prismatic.extern.hf.configuration_prismatic import (
                OpenVLAConfig,
            )
            from prismatic.extern.hf.modeling_prismatic import (
                OpenVLAForActionPrediction,
            )
            from prismatic.extern.hf.processing_prismatic import (
                PrismaticImageProcessor,
                PrismaticProcessor,
            )

        except Exception as exc:
            self.get_logger().warning(
                "Local OpenVLA registration skipped; "
                "relying on trust_remote_code. "
                f"reason={exc}"
            )
            return

        registrations = (
            (
                AutoConfig.register,
                (
                    "openvla",
                    OpenVLAConfig,
                ),
            ),
            (
                AutoImageProcessor.register,
                (
                    OpenVLAConfig,
                    PrismaticImageProcessor,
                ),
            ),
            (
                AutoProcessor.register,
                (
                    OpenVLAConfig,
                    PrismaticProcessor,
                ),
            ),
            (
                AutoModelForVision2Seq.register,
                (
                    OpenVLAConfig,
                    OpenVLAForActionPrediction,
                ),
            ),
        )

        for register_function, arguments in registrations:
            try:
                register_function(*arguments)
            except ValueError:
                # Registration may already exist.
                pass

    def _resolve_device(
        self,
        configured_device: str,
    ) -> str:
        normalized = configured_device.strip().lower()

        if normalized == "auto":
            if torch.cuda.is_available():
                return "cuda:0"
            return "cpu"

        if (
            normalized.startswith("cuda")
            and not torch.cuda.is_available()
        ):
            self.get_logger().warning(
                f"Requested {configured_device}, "
                "but CUDA is unavailable; using CPU."
            )
            return "cpu"

        return configured_device

    def _load_and_validate_dataset_statistics(
        self,
        model_path: Path,
    ) -> None:
        statistics_path = (
            model_path / "dataset_statistics.json"
        )

        with statistics_path.open(
            "r",
            encoding="utf-8",
        ) as statistics_file:
            norm_stats = json.load(
                statistics_file
            )

        if not isinstance(norm_stats, dict):
            raise ValueError(
                "dataset_statistics.json must "
                "contain a JSON object"
            )

        unnorm_key = str(
            self.get_parameter(
                "unnorm_key"
            ).value
        )

        available_keys = sorted(
            str(key)
            for key in norm_stats.keys()
        )

        if unnorm_key not in norm_stats:
            raise KeyError(
                f"unnorm_key '{unnorm_key}' was not "
                "found in dataset_statistics.json. "
                f"available_keys={available_keys}"
            )

        selected_stats = norm_stats[
            unnorm_key
        ]

        if not isinstance(selected_stats, dict):
            raise ValueError(
                f"Statistics for '{unnorm_key}' "
                "must be a JSON object"
            )

        self.model.norm_stats = norm_stats
        self.model.config.norm_stats = norm_stats

        self.get_logger().info(
            "Loaded dataset statistics:"
            f"selected={unnorm_key}, "
            f"available={available_keys}"
        )

    # ======================================================================
    # ROS callbacks
    # ======================================================================

    def _image_callback(
        self,
        msg: Image,
    ) -> None:
        with self.state_lock:
            self.latest_image = msg
            self.latest_image_id += 1

    def _instruction_callback(
        self,
        msg: String,
    ) -> None:
        instruction = msg.data.strip()

        if not instruction:
            return

        with self.state_lock:
            self.instruction = instruction

        self._publish_status(
            f"instruction_updated:{instruction}"
        )

    def _enable_callback(
        self,
        msg: Bool,
    ) -> None:
        requested_enabled = bool(msg.data)

        with self.state_lock:
            if requested_enabled == self.enabled:
                return

            self.enabled = requested_enabled
            self.episode_generation += 1

            self.command_outstanding = False
            self.robot_became_busy = False

            self.last_used_image_id = 0
            self.last_inference_attempt_time = 0.0

            self._reset_action_tracking_locked()

            generation = self.episode_generation

        self._publish_status(
            f"inference_enabled:"
            f"enabled={requested_enabled}, "
            f"generation={generation}"
        )

    def _ready_callback(
        self,
        msg: Bool,
    ) -> None:
        ready = bool(msg.data)
        cycle_completed = False

        with self.state_lock:
            self.robot_ready = ready

            if self.command_outstanding:
                if not ready:
                    self.robot_became_busy = True

                elif (
                    ready
                    and self.robot_became_busy
                ):
                    self.command_outstanding = False
                    self.robot_became_busy = False
                    cycle_completed = True

        if cycle_completed:
            self._publish_status(
                "robot_cycle_completed"
            )

    # ======================================================================
    # Inference scheduling
    # ======================================================================

    def _tick(self) -> None:
        now = time.monotonic()

        minimum_period = float(
            self.get_parameter(
                "minimum_period_sec"
            ).value
        )

        if minimum_period < 0.0:
            self.get_logger().error(
                "minimum_period_sec must not "
                "be negative"
            )
            return

        with self.state_lock:
            if not self.enabled:
                return

            if not self.robot_ready:
                return

            if self.inference_busy:
                return

            if self.command_outstanding:
                return

            if (
                now
                - self.last_inference_attempt_time
                < minimum_period
            ):
                return

            if self.latest_image is None:
                return

            # Do not infer twice from the same received frame.
            if (
                self.latest_image_id
                == self.last_used_image_id
            ):
                return

            image_msg = self.latest_image
            image_id = self.latest_image_id
            instruction = self.instruction
            generation = self.episode_generation

            self.last_used_image_id = image_id
            self.last_inference_attempt_time = now
            self.inference_busy = True

        threading.Thread(
            target=self._infer,
            args=(
                image_msg,
                image_id,
                instruction,
                generation,
            ),
            daemon=True,
            name="openvla_inference_worker",
        ).start()

    # ======================================================================
    # OpenVLA inference
    # ======================================================================

    @torch.inference_mode()
    def _infer(
        self,
        image_msg: Image,
        image_id: int,
        instruction: str,
        generation: int,
    ) -> None:
        try:
            rgb = self._image_msg_to_rgb_array(
                image_msg
            )

            pil_image = PILImage.fromarray(
                rgb,
                mode="RGB",
            )

            pil_image = self._apply_center_crop(
                pil_image
            )

            prompt = (
                "In: What action should the robot "
                f"take to {instruction.lower()}?\n"
                "Out:"
            )

            inputs = self.processor(
                prompt,
                pil_image,
            )

            inputs = inputs.to(
                self.device,
                dtype=self.torch_dtype,
            )

            generation_kwargs = (
                self._generation_kwargs()
            )

            action = self.model.predict_action(
                **inputs,
                unnorm_key=str(
                    self.get_parameter(
                        "unnorm_key"
                    ).value
                ),
                **generation_kwargs,
            )

            action = checked_array(
                action,
                ACTION_DIM,
                "OpenVLA action",
            )

            # Ensure the episode was not stopped or restarted
            # while GPU inference was running.
            with self.state_lock:
                result_is_current = bool(
                    self.enabled
                    and generation
                    == self.episode_generation
                )

                if result_is_current:
                    repeat_count = (
                        self._update_action_repeat_count_locked(
                            action
                        )
                    )

                    # Block subsequent inference until robot_ready
                    # changes False and then True.
                    self.command_outstanding = True
                    self.robot_became_busy = False
                else:
                    repeat_count = 0

            if not result_is_current:
                self._publish_status(
                    "inference_result_discarded:"
                    f"image_id={image_id}, "
                    f"generation={generation}"
                )
                return

            self.action_publisher.publish(
                array_message(action)
            )

            self._publish_action_valid(True)
            self._publish_status("inference_ok")

            image_stamp = self._stamp_to_seconds(
                image_msg
            )

            image_hash = self._image_hash(
                pil_image
            )

            image_statistics = (
                self._format_image_statistics(rgb)
            )

            self.get_logger().info(
                "inference_ok:"
                f"action={action.tolist()}, "
                f"instruction={instruction!r}, "
                f"image_id={image_id}, "
                f"image_stamp={image_stamp:.9f}, "
                f"image_hash={image_hash}, "
                f"{image_statistics}, "
                f"action_repeat_count={repeat_count}, "
                f"do_sample="
                f"{generation_kwargs['do_sample']}"
            )

            warning_count = int(
                self.get_parameter(
                    "repeated_action_warn_count"
                ).value
            )

            if (
                warning_count > 0
                and repeat_count >= warning_count
            ):
                self.get_logger().warning(
                    "repeated_action_detected:"
                    f"count={repeat_count}, "
                    f"epsilon="
                    f"{self.get_parameter('repeat_action_epsilon').value}, "
                    f"image_id={image_id}, "
                    f"image_hash={image_hash}, "
                    f"action={action.tolist()}"
                )

        except Exception as exc:
            self._publish_action_valid(False)

            self._publish_status(
                f"inference_error:{exc}"
            )

            self.get_logger().error(
                "inference_error:"
                f"type={type(exc).__name__}, "
                f"message={exc}, "
                f"image_id={image_id}, "
                f"instruction={instruction!r}"
            )

        finally:
            # Limit failed attempts as well as successful attempts.
            with self.state_lock:
                self.last_inference_attempt_time = (
                    time.monotonic()
                )
                self.inference_busy = False

    # ======================================================================
    # Image conversion
    # ======================================================================

    def _image_msg_to_rgb_array(
        self,
        msg: Image,
    ) -> np.ndarray:
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
            raise ValueError(
                "unsupported image encoding:"
                f"encoding={msg.encoding}"
            )

        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)

        if width <= 0 or height <= 0:
            raise ValueError(
                "invalid image dimensions:"
                f"width={width}, height={height}"
            )

        channels = channel_counts[encoding]
        row_width = width * channels

        if step < row_width:
            raise ValueError(
                f"invalid image step {step} "
                f"for width {width} and "
                f"{channels} channels"
            )

        data = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )

        expected_size = height * step

        if data.size < expected_size:
            raise ValueError(
                "image data too short:"
                f"got={data.size}, "
                f"expected={expected_size}"
            )

        rows = data[
            :expected_size
        ].reshape(
            height,
            step,
        )

        image = rows[
            :,
            :row_width,
        ].reshape(
            height,
            width,
            channels,
        )

        if encoding in (
            "rgb8",
            "8uc3",
        ):
            rgb = image

        elif encoding == "bgr8":
            rgb = image[:, :, ::-1]

        elif encoding == "rgba8":
            rgb = image[:, :, :3]

        elif encoding == "bgra8":
            rgb = image[:, :, 2::-1]

        else:
            rgb = np.repeat(
                image,
                3,
                axis=2,
            )

        return np.ascontiguousarray(rgb)

    def _apply_center_crop(
        self,
        image: PILImage.Image,
    ) -> PILImage.Image:
        crop_scale = float(
            self.get_parameter(
                "center_crop_scale"
            ).value
        )

        if (
            crop_scale <= 0.0
            or crop_scale > 1.0
        ):
            raise ValueError(
                "center_crop_scale must be "
                "in the range (0, 1]"
            )

        if crop_scale >= 0.999:
            return image

        width, height = image.size

        crop_ratio = math.sqrt(
            crop_scale
        )

        crop_width = max(
            1,
            int(
                round(
                    width * crop_ratio
                )
            ),
        )

        crop_height = max(
            1,
            int(
                round(
                    height * crop_ratio
                )
            ),
        )

        left = (
            width - crop_width
        ) // 2

        top = (
            height - crop_height
        ) // 2

        cropped = image.crop(
            (
                left,
                top,
                left + crop_width,
                top + crop_height,
            )
        )

        resampling = getattr(
            PILImage,
            "Resampling",
            PILImage,
        ).BICUBIC

        return cropped.resize(
            (width, height),
            resampling,
        )

    # ======================================================================
    # Generation settings
    # ======================================================================

    def _generation_kwargs(self) -> dict:
        do_sample = bool(
            self.get_parameter(
                "do_sample"
            ).value
        )

        kwargs = {
            "do_sample": do_sample,
        }

        if not do_sample:
            return kwargs

        temperature = float(
            self.get_parameter(
                "temperature"
            ).value
        )

        top_p = float(
            self.get_parameter(
                "top_p"
            ).value
        )

        if temperature <= 0.0:
            raise ValueError(
                "temperature must be greater "
                "than zero when do_sample=True"
            )

        if not 0.0 < top_p <= 1.0:
            raise ValueError(
                "top_p must be in the "
                "range (0, 1]"
            )

        kwargs["temperature"] = temperature
        kwargs["top_p"] = top_p

        return kwargs

    # ======================================================================
    # Action diagnostics
    # ======================================================================

    def _reset_action_tracking_locked(
        self,
    ) -> None:
        self.last_action = None
        self.action_repeat_count = 0

    def _update_action_repeat_count_locked(
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
                "repeat_action_epsilon must "
                "not be negative"
            )

        if (
            self.last_action is not None
            and np.allclose(
                action,
                self.last_action,
                rtol=0.0,
                atol=epsilon,
            )
        ):
            self.action_repeat_count += 1
        else:
            self.action_repeat_count = 1

        self.last_action = action.copy()

        return self.action_repeat_count

    # ======================================================================
    # Image diagnostics
    # ======================================================================

    def _stamp_to_seconds(
        self,
        msg: Image,
    ) -> float:
        return (
            float(msg.header.stamp.sec)
            + float(
                msg.header.stamp.nanosec
            )
            * 1.0e-9
        )

    def _image_hash(
        self,
        image: PILImage.Image,
    ) -> str:
        if not bool(
            self.get_parameter(
                "log_image_hash"
            ).value
        ):
            return "disabled"

        rgb = np.asarray(
            image.convert("RGB"),
            dtype=np.uint8,
        )

        return hashlib.blake2s(
            rgb.tobytes(),
            digest_size=8,
        ).hexdigest()

    def _format_image_statistics(
        self,
        rgb: np.ndarray,
    ) -> str:
        if not bool(
            self.get_parameter(
                "log_image_statistics"
            ).value
        ):
            return "image_statistics=disabled"

        return (
            f"image_shape={rgb.shape}, "
            f"image_min={int(rgb.min())}, "
            f"image_max={int(rgb.max())}, "
            f"image_mean={float(rgb.mean()):.3f}, "
            f"image_std={float(rgb.std()):.3f}"
        )

    # ======================================================================
    # Parameter logging
    # ======================================================================

    def _log_parameters(self) -> None:
        self.get_logger().info(
            "OpenVLAInference initialized:"
            f"model_path="
            f"{self.get_parameter('model_path').value}, "
            f"unnorm_key="
            f"{self.get_parameter('unnorm_key').value}, "
            f"device="
            f"{self.get_parameter('device').value}, "
            f"use_flash_attention="
            f"{self.get_parameter('use_flash_attention').value}, "
            f"minimum_period_sec="
            f"{self.get_parameter('minimum_period_sec').value}, "
            f"center_crop_scale="
            f"{self.get_parameter('center_crop_scale').value}, "
            f"do_sample="
            f"{self.get_parameter('do_sample').value}, "
            f"repeat_action_epsilon="
            f"{self.get_parameter('repeat_action_epsilon').value}, "
            f"repeated_action_warn_count="
            f"{self.get_parameter('repeated_action_warn_count').value}"
        )


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


if __name__ == "__main__":
    main()