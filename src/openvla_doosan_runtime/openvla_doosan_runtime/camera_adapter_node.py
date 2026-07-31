from __future__ import annotations

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


class CameraAdapterNode(Node):
    """
    ZED RGB image relay for OpenVLA.

    This node does not resize or crop the image.
    It only converts the incoming ROS image to RGB8.
    """

    def __init__(self) -> None:
        super().__init__("camera_adapter")

        self.declare_parameter(
            "input_topic",
            "/zed/zed_node/rgb/color/rect/image",
        )
        self.declare_parameter(
            "output_topic",
            "/vla/image_rgb",
        )

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.bridge = CvBridge()
        self.first_frame_logged = False

        self.publisher = self.create_publisher(
            Image,
            output_topic,
            image_qos,
        )
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            image_qos,
        )

        self.get_logger().info(
            f"Camera relay: {input_topic} -> {output_topic}; "
            "resize=disabled, crop=disabled"
        )

    def _image_callback(self, msg: Image) -> None:
        try:
            # cv_bridge performs only encoding conversion here.
            rgb = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="rgb8",
            )

            output = self.bridge.cv2_to_imgmsg(
                rgb,
                encoding="rgb8",
            )
            output.header = msg.header
            self.publisher.publish(output)

            if not self.first_frame_logged:
                self.first_frame_logged = True
                self.get_logger().info(
                    "First camera frame relayed | "
                    f"encoding={msg.encoding} | "
                    f"shape={tuple(rgb.shape)} | "
                    f"frame_id={msg.header.frame_id} | "
                    f"stamp={msg.header.stamp.sec}."
                    f"{msg.header.stamp.nanosec:09d}"
                )

        except CvBridgeError as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")


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
