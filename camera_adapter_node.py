from __future__ import annotations

import hashlib
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraAdapterNode(Node):
    """Convert a ZED image topic to the RGB image consumed by OpenVLA."""

    def __init__(self) -> None:
        super().__init__("camera_adapter")

        self.declare_parameter("input_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("output_topic", "/vla/image_rgb")
        self.declare_parameter("output_width", 224)
        self.declare_parameter("output_height", 224)
        self.declare_parameter("center_crop_square", True)
        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("log_interval_sec", 5.0)
        self.declare_parameter("log_image_hash", True)
        self.declare_parameter("expected_fps", 30.0)
        self.declare_parameter("frame_drop_warn_factor", 1.8)
        self.declare_parameter("slow_processing_warn_ms", 20.0)

        # Cache parameters once. Runtime parameter changes require node restart.
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_width = int(self.get_parameter("output_width").value)
        self.output_height = int(self.get_parameter("output_height").value)
        self.center_crop_square = bool(
            self.get_parameter("center_crop_square").value
        )
        self.flip_horizontal = bool(
            self.get_parameter("flip_horizontal").value
        )
        self.log_interval_sec = float(
            self.get_parameter("log_interval_sec").value
        )
        self.log_image_hash = bool(
            self.get_parameter("log_image_hash").value
        )
        self.expected_fps = float(self.get_parameter("expected_fps").value)
        self.frame_drop_warn_factor = float(
            self.get_parameter("frame_drop_warn_factor").value
        )
        self.slow_processing_warn_ms = float(
            self.get_parameter("slow_processing_warn_ms").value
        )

        self._validate_parameters()

        self.bridge = CvBridge()
        self.frame_count = 0
        self.error_count = 0
        self.frame_gap_warning_count = 0
        self.slow_processing_warning_count = 0
        self.first_frame_logged = False

        self.previous_stamp_sec: Optional[float] = None
        self.previous_periodic_hash: Optional[str] = None

        self.window_start = time.monotonic()
        self.window_frame_count = 0
        self.window_processing_ms_sum = 0.0
        self.window_processing_ms_max = 0.0
        self.window_frame_dt_ms_max = 0.0

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.publisher = self.create_publisher(
            Image,
            self.output_topic,
            qos,
        )
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self._image_callback,
            qos,
        )

        self.get_logger().info(
            "CameraAdapter ready | "
            f"input={self.input_topic} | "
            f"output={self.output_topic} | "
            f"size={self.output_width}x{self.output_height} | "
            f"crop_square={self.center_crop_square} | "
            f"flip_horizontal={self.flip_horizontal} | "
            f"expected_fps={self.expected_fps:.2f}"
        )

    def _validate_parameters(self) -> None:
        if self.output_width <= 0 or self.output_height <= 0:
            raise ValueError(
                "output_width and output_height must be greater than zero"
            )
        if self.log_interval_sec < 0.0:
            raise ValueError("log_interval_sec must not be negative")
        if self.expected_fps <= 0.0:
            raise ValueError("expected_fps must be greater than zero")
        if self.frame_drop_warn_factor <= 1.0:
            raise ValueError("frame_drop_warn_factor must be greater than 1.0")
        if self.slow_processing_warn_ms < 0.0:
            raise ValueError("slow_processing_warn_ms must not be negative")

    def _image_callback(self, msg: Image) -> None:
        callback_start = time.perf_counter()

        try:
            bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            if bgr is None:
                raise ValueError("cv_bridge returned None")
            if bgr.ndim != 3 or bgr.shape[2] != 3:
                raise ValueError(
                    "Expected HxWx3 BGR image, "
                    f"got shape={getattr(bgr, 'shape', None)}"
                )

            input_shape = tuple(bgr.shape)

            if self.center_crop_square:
                bgr = self._center_crop_square(bgr)

            if self.flip_horizontal:
                bgr = cv2.flip(bgr, 1)

            if (
                bgr.shape[1] != self.output_width
                or bgr.shape[0] != self.output_height
            ):
                interpolation = self._select_interpolation(
                    source_width=bgr.shape[1],
                    source_height=bgr.shape[0],
                    target_width=self.output_width,
                    target_height=self.output_height,
                )
                bgr = cv2.resize(
                    bgr,
                    (self.output_width, self.output_height),
                    interpolation=interpolation,
                )

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            if not rgb.flags["C_CONTIGUOUS"]:
                rgb = np.ascontiguousarray(rgb)

            output = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
            output.header = msg.header
            self.publisher.publish(output)

            self.frame_count += 1
            frame_dt_ms = self._calculate_frame_interval_ms(msg)
            processing_ms = (
                time.perf_counter() - callback_start
            ) * 1000.0
            image_hash = (
                self._calculate_image_hash(rgb)
                if self.log_image_hash
                else None
            )

            self._update_window(processing_ms, frame_dt_ms)
            self._log_first_frame(
                msg=msg,
                input_shape=input_shape,
                output_shape=tuple(rgb.shape),
                processing_ms=processing_ms,
                image_hash=image_hash,
            )
            self._warn_if_abnormal(processing_ms, frame_dt_ms)
            self._log_periodic(image_hash)

        except Exception as exc:
            self.error_count += 1
            self.get_logger().error(
                "Camera conversion failed: "
                f"{type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0,
            )

    @staticmethod
    def _center_crop_square(image: np.ndarray) -> np.ndarray:
        height, width = image.shape[:2]
        side = min(height, width)
        y0 = (height - side) // 2
        x0 = (width - side) // 2
        return image[y0:y0 + side, x0:x0 + side]

    @staticmethod
    def _select_interpolation(
        source_width: int,
        source_height: int,
        target_width: int,
        target_height: int,
    ) -> int:
        if target_width < source_width or target_height < source_height:
            return cv2.INTER_AREA
        return cv2.INTER_LINEAR

    @staticmethod
    def _calculate_image_hash(image: np.ndarray) -> str:
        return hashlib.sha256(image.tobytes()).hexdigest()[:8]

    def _calculate_frame_interval_ms(
        self,
        msg: Image,
    ) -> Optional[float]:
        stamp_sec = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )

        if self.previous_stamp_sec is None:
            frame_dt_ms = None
        else:
            frame_dt_ms = (stamp_sec - self.previous_stamp_sec) * 1000.0
            if frame_dt_ms <= 0.0:
                self.get_logger().warning(
                    "Non-increasing camera timestamp | "
                    f"previous={self.previous_stamp_sec:.9f} | "
                    f"current={stamp_sec:.9f}"
                )

        self.previous_stamp_sec = stamp_sec
        return frame_dt_ms

    def _update_window(
        self,
        processing_ms: float,
        frame_dt_ms: Optional[float],
    ) -> None:
        self.window_frame_count += 1
        self.window_processing_ms_sum += processing_ms
        self.window_processing_ms_max = max(
            self.window_processing_ms_max,
            processing_ms,
        )
        if frame_dt_ms is not None:
            self.window_frame_dt_ms_max = max(
                self.window_frame_dt_ms_max,
                frame_dt_ms,
            )

    def _warn_if_abnormal(
        self,
        processing_ms: float,
        frame_dt_ms: Optional[float],
    ) -> None:
        expected_period_ms = 1000.0 / self.expected_fps
        gap_threshold_ms = (
            expected_period_ms * self.frame_drop_warn_factor
        )

        if frame_dt_ms is not None and frame_dt_ms > gap_threshold_ms:
            self.frame_gap_warning_count += 1
            estimated_missing = max(
                1,
                int(round(frame_dt_ms / expected_period_ms)) - 1,
            )
            self.get_logger().warning(
                "Camera frame gap | "
                f"dt_ms={frame_dt_ms:.2f} | "
                f"expected_ms={expected_period_ms:.2f} | "
                f"estimated_missing={estimated_missing}"
            )

        if (
            self.slow_processing_warn_ms > 0.0
            and processing_ms > self.slow_processing_warn_ms
        ):
            self.slow_processing_warning_count += 1
            self.get_logger().warning(
                "Slow camera processing | "
                f"processing_ms={processing_ms:.2f} | "
                f"threshold_ms={self.slow_processing_warn_ms:.2f}"
            )

    def _log_first_frame(
        self,
        msg: Image,
        input_shape: Tuple[int, ...],
        output_shape: Tuple[int, ...],
        processing_ms: float,
        image_hash: Optional[str],
    ) -> None:
        if self.first_frame_logged:
            return

        self.first_frame_logged = True
        fields = [
            "First camera frame",
            f"encoding={msg.encoding}",
            f"input_shape={input_shape}",
            f"output_shape={output_shape}",
            f"frame_id={msg.header.frame_id}",
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
            f"processing_ms={processing_ms:.2f}",
        ]
        if image_hash is not None:
            fields.append(f"image_hash={image_hash}")

        self.get_logger().info(" | ".join(fields))

    def _log_periodic(self, image_hash: Optional[str]) -> None:
        if self.log_interval_sec <= 0.0:
            return

        now = time.monotonic()
        elapsed_sec = now - self.window_start
        if elapsed_sec < self.log_interval_sec:
            return

        count = self.window_frame_count
        fps = count / elapsed_sec if elapsed_sec > 0.0 else 0.0
        average_processing_ms = (
            self.window_processing_ms_sum / count if count > 0 else 0.0
        )

        fields = [
            f"frames_total={self.frame_count}",
            f"fps={fps:.2f}",
            f"processing_avg_ms={average_processing_ms:.2f}",
            f"processing_max_ms={self.window_processing_ms_max:.2f}",
            f"frame_dt_max_ms={self.window_frame_dt_ms_max:.2f}",
            f"frame_gap_warnings={self.frame_gap_warning_count}",
            f"slow_warnings={self.slow_processing_warning_count}",
            f"errors={self.error_count}",
        ]

        if image_hash is not None:
            image_changed = (
                None
                if self.previous_periodic_hash is None
                else image_hash != self.previous_periodic_hash
            )
            fields.append(f"image_hash={image_hash}")
            fields.append(f"image_changed={image_changed}")
            self.previous_periodic_hash = image_hash

        self.get_logger().info(
            "Camera diagnostics | " + " | ".join(fields)
        )

        self.window_start = now
        self.window_frame_count = 0
        self.window_processing_ms_sum = 0.0
        self.window_processing_ms_max = 0.0
        self.window_frame_dt_ms_max = 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[CameraAdapterNode] = None

    try:
        node = CameraAdapterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(
                "CameraAdapter terminated: "
                f"{type(exc).__name__}: {exc}"
            )
        else:
            print(
                "CameraAdapter failed to start: "
                f"{type(exc).__name__}: {exc}"
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
