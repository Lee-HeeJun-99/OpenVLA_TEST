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
from cv_bridge import CvBridge
from PIL import Image
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Bool, Float64MultiArray, String
from transformers import AutoModelForVision2Seq, AutoProcessor


ACTION_DIM = 7


class OpenVLAInferenceNode(Node):
    """
    OpenVLA ROS2 inference node.

    입력 토픽
    ----------
    /vla/image_rgb
        sensor_msgs/msg/Image

    /vla/instruction
        std_msgs/msg/String

    /vla/enable
        std_msgs/msg/Bool

    /vla/robot_ready
        std_msgs/msg/Bool

    /vla/step_done
        std_msgs/msg/Bool
        (doosan_bridge_node가 한 스텝의 이동+그리퍼가
        완전히 끝났을 때 정확히 한 번 발행하는 이벤트)

    출력 토픽
    ----------
    /vla/raw_action
        std_msgs/msg/Float64MultiArray

    /vla/action_valid
        std_msgs/msg/Bool

    /vla/model_status
        std_msgs/msg/String

    require_robot_cycle=True
        실제 로봇 모드.

        robot_ready=True
        → action 발행 (command_outstanding=True)
        → doosan_bridge가 이동+그리퍼를 실행
        → /vla/step_done 이벤트 수신 (command_outstanding=False)
        → 다음 추론 허용

        완료 판정은 robot_ready의 폴링된 값이 아니라
        /vla/step_done 이벤트로만 한다. robot_ready는
        폴링(0.1초 간격) 방식이라 폴링 주기보다 짧게 끝나는
        동작의 false 구간을 놓칠 수 있고, 그러면
        command_outstanding이 영원히 안 풀리는 데드락이
        발생하기 때문이다. command_outstanding_timeout_sec는
        혹시 step_done 메시지 자체가 유실됐을 때를 대비한
        watchdog이다.

    require_robot_cycle=False
        rosbag 테스트 모드.

        enable=True, robot_ready=True 상태에서 새 이미지가 들어올 때마다
        minimum_period_sec 간격으로 계속 추론한다.
    """

    def __init__(self) -> None:
        super().__init__("openvla_inference")

        # ================================================================
        # Parameters
        # ================================================================

        self.declare_parameter("model_path", "")
        self.declare_parameter("unnorm_key", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("use_flash_attention", True)

        self.declare_parameter(
            "default_instruction",
            "pick up the cube",
        )

        self.declare_parameter(
            "minimum_period_sec",
            0.1,
        )

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

        # 실제 로봇과 rosbag 모드를 구분하는 핵심 파라미터
        self.declare_parameter(
            "require_robot_cycle",
            True,
        )

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

        # step_done 이벤트가 유실됐을 때를 대비한 watchdog 타임아웃
        self.declare_parameter(
            "command_outstanding_timeout_sec",
            2.0,
        )

        # ================================================================
        # Read parameters
        # ================================================================

        self.model_path = str(
            self.get_parameter("model_path").value
        )

        self.unnorm_key = str(
            self.get_parameter("unnorm_key").value
        )

        self.device_name = str(
            self.get_parameter("device").value
        )

        self.use_flash_attention = bool(
            self.get_parameter(
                "use_flash_attention"
            ).value
        )

        self.minimum_period_sec = max(
            0.0,
            float(
                self.get_parameter(
                    "minimum_period_sec"
                ).value
            ),
        )

        self.center_crop_scale = float(
            self.get_parameter(
                "center_crop_scale"
            ).value
        )

        self.do_sample = bool(
            self.get_parameter(
                "do_sample"
            ).value
        )

        self.temperature = float(
            self.get_parameter(
                "temperature"
            ).value
        )

        self.top_p = float(
            self.get_parameter(
                "top_p"
            ).value
        )

        self.require_robot_cycle = bool(
            self.get_parameter(
                "require_robot_cycle"
            ).value
        )

        self.log_image_hash = bool(
            self.get_parameter(
                "log_image_hash"
            ).value
        )

        self.log_image_statistics = bool(
            self.get_parameter(
                "log_image_statistics"
            ).value
        )

        self.repeat_action_epsilon = max(
            0.0,
            float(
                self.get_parameter(
                    "repeat_action_epsilon"
                ).value
            ),
        )

        self.repeated_action_warn_count = max(
            1,
            int(
                self.get_parameter(
                    "repeated_action_warn_count"
                ).value
            ),
        )

        self.command_outstanding_timeout_sec = max(
            0.0,
            float(
                self.get_parameter(
                    "command_outstanding_timeout_sec"
                ).value
            ),
        )

        if not 0.0 < self.center_crop_scale <= 1.0:
            raise ValueError(
                "center_crop_scale must be greater than 0 "
                "and less than or equal to 1"
            )

        # ================================================================
        # Shared state
        # ================================================================

        self._lock = threading.RLock()
        self._bridge = CvBridge()

        self.enabled = False
        self.robot_ready = False

        self.instruction = str(
            self.get_parameter(
                "default_instruction"
            ).value
        ).strip()

        self.latest_image: Optional[Image.Image] = None
        self.latest_image_stamp = ""

        self.latest_image_id = 0
        self.last_used_image_id = -1

        self.inference_busy = False
        self.last_inference_attempt_time = 0.0

        # 실제 로봇 command handshake 상태
        self.command_outstanding = False
        self.robot_became_busy = False
        self.command_outstanding_since: Optional[float] = None

        # 이전 episode의 추론 결과를 폐기하기 위한 값
        self.episode_generation = 0

        self.last_action: Optional[np.ndarray] = None
        self.action_repeat_count = 0

        self.model = None
        self.processor = None

        # ================================================================
        # QoS
        # ================================================================

        instruction_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        default_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ================================================================
        # Publishers
        # ================================================================

        self.action_pub = self.create_publisher(
            Float64MultiArray,
            "/vla/raw_action",
            default_qos,
        )

        self.action_valid_pub = self.create_publisher(
            Bool,
            "/vla/action_valid",
            default_qos,
        )

        self.model_status_pub = self.create_publisher(
            String,
            "/vla/model_status",
            instruction_qos,
        )

        # ================================================================
        # Subscribers
        # ================================================================

        self.create_subscription(
            RosImage,
            "/vla/image_rgb",
            self._image_callback,
            sensor_qos,
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
            self._robot_ready_callback,
            default_qos,
        )

        self.create_subscription(
            Bool,
            "/vla/step_done",
            self._step_done_callback,
            default_qos,
        )

        # 100 Hz 조건 검사
        self._timer = self.create_timer(
            0.01,
            self._tick,
        )

        self._publish_status("loading")
        self._load_model()
        self._publish_status("ready")

        self.get_logger().info(
            "OpenVLA inference node ready | "
            f"require_robot_cycle={self.require_robot_cycle} | "
            f"minimum_period_sec={self.minimum_period_sec:.3f} | "
            f"unnorm_key='{self.unnorm_key}'"
        )

    # ================================================================
    # Model loading
    # ================================================================

    def _load_model(self) -> None:
        if not self.model_path:
            raise ValueError(
                "model_path parameter is empty"
            )

        model_dir = Path(
            self.model_path
        ).expanduser().resolve()

        if not model_dir.is_dir():
            raise FileNotFoundError(
                f"Model directory does not exist: {model_dir}"
            )

        required_files = (
            "config.json",
            "model.safetensors.index.json",
            "processor_config.json",
            "dataset_statistics.json",
        )

        missing_files = [
            filename
            for filename in required_files
            if not (model_dir / filename).is_file()
        ]

        if missing_files:
            raise FileNotFoundError(
                "Missing checkpoint files: "
                + ", ".join(missing_files)
            )

        statistics_path = (
            model_dir / "dataset_statistics.json"
        )

        try:
            with statistics_path.open(
                "r",
                encoding="utf-8",
            ) as file:
                statistics = json.load(file)

        except Exception as exc:
            raise RuntimeError(
                f"Failed to read {statistics_path}: {exc}"
            ) from exc

        if not self.unnorm_key:
            available_keys = list(
                statistics.keys()
            )

            if len(available_keys) == 1:
                self.unnorm_key = available_keys[0]

                self.get_logger().warning(
                    "unnorm_key is empty. "
                    f"Using '{self.unnorm_key}'"
                )

            else:
                raise KeyError(
                    "unnorm_key is empty. "
                    f"Available keys: {available_keys}"
                )

        if self.unnorm_key not in statistics:
            raise KeyError(
                f"unnorm_key '{self.unnorm_key}' was not found. "
                f"Available keys: {list(statistics.keys())}"
            )

        if (
            self.device_name.startswith("cuda")
            and not torch.cuda.is_available()
        ):
            raise RuntimeError(
                f"CUDA device '{self.device_name}' requested, "
                "but CUDA is unavailable"
            )

        if self.device_name.startswith("cuda"):
            torch_dtype = torch.bfloat16
        else:
            torch_dtype = torch.float32

        model_kwargs = {
            "trust_remote_code": True,
            "torch_dtype": torch_dtype,
            "low_cpu_mem_usage": True,
        }

        if (
            self.use_flash_attention
            and self.device_name.startswith("cuda")
        ):
            model_kwargs[
                "attn_implementation"
            ] = "flash_attention_2"

        self.get_logger().info(
            f"Loading processor: {model_dir}"
        )

        self.processor = AutoProcessor.from_pretrained(
            str(model_dir),
            trust_remote_code=True,
        )

        self.get_logger().info(
            f"Loading model: {model_dir}"
        )

        try:
            self.model = (
                AutoModelForVision2Seq.from_pretrained(
                    str(model_dir),
                    **model_kwargs,
                )
            )

        except Exception as exc:
            if "attn_implementation" in model_kwargs:
                self.get_logger().warning(
                    "Flash Attention loading failed. "
                    f"Retrying without Flash Attention: {exc}"
                )

                model_kwargs.pop(
                    "attn_implementation",
                    None,
                )

                self.model = (
                    AutoModelForVision2Seq.from_pretrained(
                        str(model_dir),
                        **model_kwargs,
                    )
                )

            else:
                raise

        self.model.norm_stats = statistics

        if hasattr(
            self.model,
            "config",
        ):
            self.model.config.norm_stats = statistics

        self.model = self.model.to(
            self.device_name
        )

        self.model.eval()

        if not hasattr(
            self.model,
            "predict_action",
        ):
            raise AttributeError(
                "Loaded model does not provide predict_action(). "
                "Check that this is an OpenVLA checkpoint."
            )

        if hasattr(
            self.model,
            "get_action_dim",
        ):
            action_dim = self.model.get_action_dim(
                self.unnorm_key
            )

            if action_dim != ACTION_DIM:
                raise ValueError(
                    f"Expected {ACTION_DIM}-D action statistics "
                    f"for unnorm_key '{self.unnorm_key}', "
                    f"got {action_dim}"
                )

        self.get_logger().info(
            f"Model loaded on {self.device_name} "
            f"with dtype={torch_dtype} | "
            f"norm_stats_keys={list(statistics.keys())}"
        )

    # ================================================================
    # ROS callbacks
    # ================================================================

    def _image_callback(
        self,
        msg: RosImage,
    ) -> None:
        try:
            cv_image = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="rgb8",
            )

            np_image = np.asarray(
                cv_image,
                dtype=np.uint8,
            )

            if (
                np_image.ndim != 3
                or np_image.shape[2] != 3
            ):
                raise ValueError(
                    "Expected HxWx3 RGB image, "
                    f"got {np_image.shape}"
                )

            pil_image = Image.fromarray(
                np_image,
                mode="RGB",
            )

            pil_image = self._center_crop(
                pil_image
            )

            stamp = msg.header.stamp

            stamp_text = (
                f"{stamp.sec}."
                f"{stamp.nanosec:09d}"
            )

            with self._lock:
                self.latest_image = pil_image
                self.latest_image_stamp = stamp_text
                self.latest_image_id += 1

        except Exception as exc:
            self.get_logger().error(
                f"Image conversion failed: {exc}"
            )

            self._publish_action_valid(
                False
            )

    def _instruction_callback(
        self,
        msg: String,
    ) -> None:
        instruction = msg.data.strip()

        if not instruction:
            self.get_logger().warning(
                "Empty instruction received"
            )
            return

        with self._lock:
            self.instruction = instruction

        self.get_logger().info(
            f"Instruction updated: '{instruction}'"
        )

    def _enable_callback(
        self,
        msg: Bool,
    ) -> None:
        enabled = bool(msg.data)

        with self._lock:
            if enabled == self.enabled:
                return

            self.enabled = enabled
            self.episode_generation += 1

            self.command_outstanding = False
            self.robot_became_busy = False

            self.last_action = None
            self.action_repeat_count = 0

            if enabled:
                # 현재 이미지도 한 번 사용할 수 있도록 초기화
                self.last_used_image_id = -1

        self.get_logger().info(
            f"Inference "
            f"{'enabled' if enabled else 'disabled'} | "
            f"generation={self.episode_generation}"
        )

    def _robot_ready_callback(
        self,
        msg: Bool,
    ) -> None:
        """
        robot_ready는 0.1초 폴링으로 샘플링되는 상태값이라,
        폴링 주기보다 짧게 끝나는 동작의 false 구간을
        놓칠 수 있다 (그러면 command_outstanding이 영원히
        안 풀리는 데드락이 발생함).

        그래서 command 완료 판정은 여기서 하지 않고,
        doosan_bridge가 명시적으로 발행하는
        /vla/step_done 이벤트(_step_done_callback)에서만 한다.
        여기서는 다음 추론을 시작해도 되는지 확인하는
        robot_ready 상태값만 갱신한다.
        """
        ready = bool(msg.data)

        with self._lock:
            self.robot_ready = ready

    def _step_done_callback(
        self,
        msg: Bool,
    ) -> None:
        """
        doosan_bridge_node가 한 스텝(이동 + 그리퍼)을
        완전히 끝낸 시점에 정확히 한 번 발행하는 이벤트.
        폴링이 아니라 이벤트이므로, 동작이 아무리 짧아도
        놓칠 수 없다. command_outstanding 해제는 여기서만 한다.
        """
        with self._lock:
            if not self.require_robot_cycle:
                return

            if not self.command_outstanding:
                return

            self.command_outstanding = False
            self.robot_became_busy = False
            self.command_outstanding_since = None

            self.get_logger().debug(
                "Robot cycle completed via step_done: "
                f"success={bool(msg.data)}"
            )

    # ================================================================
    # Inference scheduler
    # ================================================================

    def _tick(self) -> None:
        now = time.monotonic()

        with self._lock:
            if not self.enabled:
                return

            if not self.robot_ready:
                return

            # step_done 유실 등 예외 상황에 대비한 watchdog.
            # command_outstanding이 비정상적으로 오래 안 풀리면
            # 강제로 리셋해서 완전한 데드락을 방지한다.
            if (
                self.command_outstanding
                and self.command_outstanding_since is not None
                and (
                    now - self.command_outstanding_since
                )
                > self.command_outstanding_timeout_sec
            ):
                self.get_logger().warning(
                    "command_outstanding stuck longer than "
                    f"{self.command_outstanding_timeout_sec}s — "
                    "forcing reset (step_done 유실 의심)"
                )
                self.command_outstanding = False
                self.robot_became_busy = False
                self.command_outstanding_since = None

            if self.inference_busy:
                return

            if (
                self.require_robot_cycle
                and self.command_outstanding
            ):
                return

            if self.latest_image is None:
                return

            # 같은 이미지는 다시 사용하지 않음
            if (
                self.latest_image_id
                == self.last_used_image_id
            ):
                return

            elapsed = (
                now
                - self.last_inference_attempt_time
            )

            if elapsed < self.minimum_period_sec:
                return

            image = self.latest_image.copy()
            image_id = self.latest_image_id
            image_stamp = self.latest_image_stamp
            instruction = self.instruction
            generation = self.episode_generation

            self.last_used_image_id = image_id
            self.inference_busy = True

        thread = threading.Thread(
            target=self._run_inference,
            args=(
                image,
                image_id,
                image_stamp,
                instruction,
                generation,
            ),
            daemon=True,
        )

        thread.start()

    # ================================================================
    # Inference
    # ================================================================

    def _run_inference(
        self,
        image: Image.Image,
        image_id: int,
        image_stamp: str,
        instruction: str,
        generation: int,
    ) -> None:
        try:
            action = self._predict_action(
                image=image,
                instruction=instruction,
            )

            action = self._validate_action(
                action
            )

            image_array = np.asarray(
                image,
                dtype=np.uint8,
            )

            image_hash = (
                self._calculate_image_hash(
                    image_array
                )
            )

            with self._lock:
                result_is_current = (
                    self.enabled
                    and generation
                    == self.episode_generation
                )

                if not result_is_current:
                    self.get_logger().warning(
                        "Discarding old inference result | "
                        f"result generation={generation} | "
                        f"current generation="
                        f"{self.episode_generation}"
                    )
                    return

                repeat_count = (
                    self._update_action_repeat_count_locked(
                        action
                    )
                )

                if self.require_robot_cycle:
                    # 실제 로봇 모드에서는 step_done 이벤트를 기다림
                    self.command_outstanding = True
                    self.robot_became_busy = False
                    self.command_outstanding_since = (
                        time.monotonic()
                    )

                else:
                    # rosbag 모드에서는 바로 다음 이미지 추론 허용
                    self.command_outstanding = False
                    self.robot_became_busy = False
                    self.command_outstanding_since = None

            self._publish_action(
                action
            )

            self._publish_action_valid(
                True
            )

            self._log_inference(
                action=action,
                image=image_array,
                image_id=image_id,
                image_stamp=image_stamp,
                image_hash=image_hash,
                instruction=instruction,
                repeat_count=repeat_count,
            )

        except Exception as exc:
            self.get_logger().error(
                "Inference failed: "
                f"{type(exc).__name__}: {exc}"
            )

            self._publish_action_valid(
                False
            )

        finally:
            with self._lock:
                self.inference_busy = False

                # 성공/실패 여부와 관계없이 재시도 간격 적용
                self.last_inference_attempt_time = (
                    time.monotonic()
                )

    def _predict_action(
        self,
        image: Image.Image,
        instruction: str,
    ) -> np.ndarray:
        if (
            self.model is None
            or self.processor is None
        ):
            raise RuntimeError(
                "Model or processor is not loaded"
            )

        prompt = (
            "In: What action should the robot take "
            f"to {instruction}?\n"
            "Out:"
        )

        inputs = self.processor(
            prompt,
            image,
            return_tensors="pt",
        )

        moved_inputs = {}

        model_dtype = next(
            self.model.parameters()
        ).dtype

        for key, value in inputs.items():
            if torch.is_tensor(value):
                if value.is_floating_point():
                    value = value.to(
                        device=self.device_name,
                        dtype=model_dtype,
                    )
                else:
                    value = value.to(
                        self.device_name
                    )

            moved_inputs[key] = value

        predict_kwargs = {
            "unnorm_key": self.unnorm_key,
            "do_sample": self.do_sample,
        }

        if self.do_sample:
            predict_kwargs[
                "temperature"
            ] = self.temperature

            predict_kwargs[
                "top_p"
            ] = self.top_p

        with torch.inference_mode():
            action = self.model.predict_action(
                **moved_inputs,
                **predict_kwargs,
            )

        return np.asarray(
            action,
            dtype=np.float64,
        ).reshape(-1)

    # ================================================================
    # Validation
    # ================================================================

    @staticmethod
    def _validate_action(
        action: np.ndarray,
    ) -> np.ndarray:
        action = np.asarray(
            action,
            dtype=np.float64,
        ).reshape(-1)

        if action.size != ACTION_DIM:
            raise ValueError(
                f"Expected {ACTION_DIM}-D action, "
                f"got shape {action.shape}"
            )

        if not np.all(
            np.isfinite(action)
        ):
            raise ValueError(
                "Action contains NaN or Inf: "
                f"{action.tolist()}"
            )

        return action

    def _update_action_repeat_count_locked(
        self,
        action: np.ndarray,
    ) -> int:
        if self.last_action is None:
            self.action_repeat_count = 0

        elif np.allclose(
            action,
            self.last_action,
            rtol=0.0,
            atol=self.repeat_action_epsilon,
        ):
            self.action_repeat_count += 1

        else:
            self.action_repeat_count = 0

        self.last_action = action.copy()

        return self.action_repeat_count

    @staticmethod
    def _calculate_image_hash(
        image: np.ndarray,
    ) -> str:
        return hashlib.sha256(
            image.tobytes()
        ).hexdigest()[:16]

    # ================================================================
    # Logging
    # ================================================================

    def _log_inference(
        self,
        action: np.ndarray,
        image: np.ndarray,
        image_id: int,
        image_stamp: str,
        image_hash: str,
        instruction: str,
        repeat_count: int,
    ) -> None:
        fields = [
            f"image_id={image_id}",
            f"stamp={image_stamp}",
            f"instruction='{instruction}'",
            (
                "action="
                + np.array2string(
                    action,
                    precision=6,
                )
            ),
            f"repeat_count={repeat_count}",
        ]

        if self.log_image_hash:
            fields.append(
                f"image_hash={image_hash}"
            )

        if self.log_image_statistics:
            fields.extend(
                [
                    f"shape={tuple(image.shape)}",
                    f"min={int(image.min())}",
                    f"max={int(image.max())}",
                    (
                        f"mean="
                        f"{float(image.mean()):.3f}"
                    ),
                    (
                        f"std="
                        f"{float(image.std()):.3f}"
                    ),
                ]
            )

        log_text = " | ".join(fields)

        if (
            repeat_count
            >= self.repeated_action_warn_count
        ):
            self.get_logger().warning(
                log_text
            )

        else:
            self.get_logger().info(
                log_text
            )

    # ================================================================
    # Image preprocessing
    # ================================================================

    def _center_crop(
        self,
        image: Image.Image,
    ) -> Image.Image:
        width, height = image.size

        # Match the training pipeline more closely:
        # first remove aspect-ratio distortion with a centered square crop,
        # then apply the configured scale (e.g. 0.9) inside that square.
        crop_size = min(width, height)

        if not math.isclose(self.center_crop_scale, 1.0):
            crop_size = max(
                1,
                int(
                    crop_size
                    * self.center_crop_scale
                ),
            )

        left = (width - crop_size) // 2
        top = (height - crop_size) // 2

        return image.crop(
            (
                left,
                top,
                left + crop_size,
                top + crop_size,
            )
        )

    # ================================================================
    # Publishers
    # ================================================================

    def _publish_action(
        self,
        action: np.ndarray,
    ) -> None:
        msg = Float64MultiArray()

        msg.data = action.astype(
            float
        ).tolist()

        self.action_pub.publish(
            msg
        )

    def _publish_action_valid(
        self,
        valid: bool,
    ) -> None:
        msg = Bool()
        msg.data = bool(valid)

        self.action_valid_pub.publish(
            msg
        )

    def _publish_status(
        self,
        status: str,
    ) -> None:
        msg = String()
        msg.data = status

        self.model_status_pub.publish(
            msg
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[
        OpenVLAInferenceNode
    ] = None

    try:
        node = OpenVLAInferenceNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(
                "OpenVLA inference node terminated: "
                f"{exc}"
            )
        else:
            print(
                "OpenVLA inference node failed "
                f"to start: {exc}"
            )

        raise

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
