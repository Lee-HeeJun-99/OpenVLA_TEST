from __future__ import annotations

from typing import Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraAdapterNode(Node):
    """
    Converts the ZED image topic into the RGB image topic consumed by OpenVLA.

    Processing order:
      1. ROS Image -> OpenCV BGR
      2. Optional center square crop
      3. Optional horizontal flip
      4. Resize
      5. BGR -> RGB
      6. Publish as rgb8

    The original ROS timestamp and frame ID are preserved.
    """

    def __init__(self) -> None:
        super().__init__("camera_adapter")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter(
            "input_topic",
            "/zed/zed_node/rgb/color/rect/image",
        )
        self.declare_parameter(
            "output_topic",
            "/vla/image_rgb",
        )
        self.declare_parameter("output_width", 640)
        self.declare_parameter("output_height", 480)
        self.declare_parameter("center_crop_square", False)
        self.declare_parameter("flip_horizontal", False)

        # Publish a diagnostic log every N received frames.
        # Set to 0 to disable periodic frame logs.
        self.declare_parameter("log_every_n_frames", 100)

        input_topic = str(
            self.get_parameter("input_topic").value
        )
        output_topic = str(
            self.get_parameter("output_topic").value
        )

        self._validate_parameters()

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.bridge = CvBridge()
        self.frame_count = 0
        self.first_frame_logged = False
        self.last_input_shape: Optional[Tuple[int, ...]] = None

        # Camera topics commonly use sensor-data style QoS.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self.publisher = self.create_publisher(
            Image,
            output_topic,
            qos,
        )

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            qos,
        )

        self.get_logger().info(
            "CameraAdapter initialized:"
            f"input_topic={input_topic}, "
            f"output_topic={output_topic}, "
            f"output_size="
            f"{self.get_parameter('output_width').value}x"
            f"{self.get_parameter('output_height').value}, "
            f"center_crop_square="
            f"{self.get_parameter('center_crop_square').value}, "
            f"flip_horizontal="
            f"{self.get_parameter('flip_horizontal').value}"
        )

    def _validate_parameters(self) -> None:
        width = int(
            self.get_parameter("output_width").value
        )
        height = int(
            self.get_parameter("output_height").value
        )

        if width <= 0:
            raise ValueError(
                f"output_width must be greater than zero: {width}"
            )

        if height <= 0:
            raise ValueError(
                f"output_height must be greater than zero: {height}"
            )

        log_every_n_frames = int(
            self.get_parameter("log_every_n_frames").value
        )

        if log_every_n_frames < 0:
            raise ValueError(
                "log_every_n_frames must not be negative:"
                f"{log_every_n_frames}"
            )

    def _image_callback(self, msg: Image) -> None:
        try:
            self.frame_count += 1

            # ZED color topics usually use bgra8 or bgr8 depending on
            # configuration. CvBridge converts supported encodings to bgr8.
            bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            if bgr is None:
                raise ValueError("cv_bridge returned None")

            if bgr.ndim != 3 or bgr.shape[2] != 3:
                raise ValueError(
                    "Expected a 3-channel image:"
                    f"shape={getattr(bgr, 'shape', None)}"
                )

            input_shape = bgr.shape
            self.last_input_shape = input_shape

            # --------------------------------------------------------------
            # Optional center square crop
            # --------------------------------------------------------------
            if bool(
                self.get_parameter("center_crop_square").value
            ):
                bgr = self._center_crop_square(bgr)

            processed_shape = bgr.shape

            # --------------------------------------------------------------
            # Optional horizontal flip
            # --------------------------------------------------------------
            if bool(
                self.get_parameter("flip_horizontal").value
            ):
                bgr = cv2.flip(bgr, 1)

            # --------------------------------------------------------------
            # Resize
            # --------------------------------------------------------------
            output_width = int(
                self.get_parameter("output_width").value
            )
            output_height = int(
                self.get_parameter("output_height").value
            )

            self._validate_output_size(
                output_width,
                output_height,
            )

            interpolation = self._select_interpolation(
                source_width=bgr.shape[1],
                source_height=bgr.shape[0],
                target_width=output_width,
                target_height=output_height,
            )

            if (
                bgr.shape[1] != output_width
                or bgr.shape[0] != output_height
            ):
                bgr = cv2.resize(
                    bgr,
                    (output_width, output_height),
                    interpolation=interpolation,
                )

            # --------------------------------------------------------------
            # BGR -> RGB
            # --------------------------------------------------------------
            rgb = cv2.cvtColor(
                bgr,
                cv2.COLOR_BGR2RGB,
            )

            # Ensure the image memory is contiguous before publishing.
            if not rgb.flags["C_CONTIGUOUS"]:
                rgb = rgb.copy()

            output = self.bridge.cv2_to_imgmsg(
                rgb,
                encoding="rgb8",
            )

            # Preserve camera timestamp and frame ID.
            output.header = msg.header

            self.publisher.publish(output)

            self._log_frame_information(
                msg=msg,
                input_shape=input_shape,
                processed_shape=processed_shape,
                output_shape=rgb.shape,
            )

        except Exception as exc:
            self.get_logger().error(
                f"Image conversion failed: {exc}",
                throttle_duration_sec=2.0,
            )

    @staticmethod
    def _center_crop_square(image):
        height, width = image.shape[:2]
        side = min(height, width)

        y0 = (height - side) // 2
        x0 = (width - side) // 2

        return image[
            y0 : y0 + side,
            x0 : x0 + side,
        ]

    @staticmethod
    def _select_interpolation(
        source_width: int,
        source_height: int,
        target_width: int,
        target_height: int,
    ) -> int:
        """
        INTER_AREA is generally appropriate for downscaling.
        INTER_LINEAR is generally appropriate for upscaling.
        """

        if (
            target_width < source_width
            or target_height < source_height
        ):
            return cv2.INTER_AREA

        return cv2.INTER_LINEAR

    @staticmethod
    def _validate_output_size(
        width: int,
        height: int,
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError(
                "Invalid output image size:"
                f"width={width}, height={height}"
            )

    def _log_frame_information(
        self,
        msg: Image,
        input_shape: Tuple[int, ...],
        processed_shape: Tuple[int, ...],
        output_shape: Tuple[int, ...],
    ) -> None:
        if not self.first_frame_logged:
            self.first_frame_logged = True

            self.get_logger().info(
                "First camera frame:"
                f"input_encoding={msg.encoding}, "
                f"input_shape={input_shape}, "
                f"processed_shape={processed_shape}, "
                f"output_shape={output_shape}, "
                f"frame_id={msg.header.frame_id}, "
                f"stamp="
                f"{msg.header.stamp.sec}."
                f"{msg.header.stamp.nanosec:09d}"
            )

        log_every_n_frames = int(
            self.get_parameter("log_every_n_frames").value
        )

        if (
            log_every_n_frames > 0
            and self.frame_count % log_every_n_frames == 0
        ):
            self.get_logger().info(
                "Camera frames processed:"
                f"count={self.frame_count}, "
                f"input_shape={input_shape}, "
                f"output_shape={output_shape}, "
                f"stamp="
                f"{msg.header.stamp.sec}."
                f"{msg.header.stamp.nanosec:09d}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = CameraAdapterNode()

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
