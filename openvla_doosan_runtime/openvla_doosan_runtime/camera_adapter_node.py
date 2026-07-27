\
from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraAdapterNode(Node):
    """Converts the ZED image topic to the RGB image topic consumed by OpenVLA."""

    def __init__(self) -> None:
        super().__init__("camera_adapter")

        self.declare_parameter(
            "input_topic",
            "/zed/zed_node/rgb/color/rect/image",
        )
        self.declare_parameter("output_topic", "/vla/image_rgb")
        self.declare_parameter("output_width", 640)
        self.declare_parameter("output_height", 480)
        self.declare_parameter("center_crop_square", False)
        self.declare_parameter("flip_horizontal", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, output_topic, qos)
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            qos,
        )
        self.get_logger().info(f"Camera adapter: {input_topic} -> {output_topic}")

    def _image_callback(self, msg: Image) -> None:
        try:
            # ZED ROS topics normally use BGR8. cv_bridge also handles other encodings.
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if bool(self.get_parameter("center_crop_square").value):
                height, width = bgr.shape[:2]
                side = min(height, width)
                y0 = (height - side) // 2
                x0 = (width - side) // 2
                bgr = bgr[y0 : y0 + side, x0 : x0 + side]

            if bool(self.get_parameter("flip_horizontal").value):
                bgr = cv2.flip(bgr, 1)

            width = int(self.get_parameter("output_width").value)
            height = int(self.get_parameter("output_height").value)
            bgr = cv2.resize(bgr, (width, height), interpolation=cv2.INTER_AREA)

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            output = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
            output.header = msg.header
            self.publisher.publish(output)
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
