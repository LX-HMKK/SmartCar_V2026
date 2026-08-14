"""Publish a single image repeatedly with fresh ROS timestamps."""
import math
from pathlib import Path


def resolve_image_file(value):
    path = Path(str(value)).expanduser()
    if not str(value).strip():
        raise ValueError("image_file must be provided")
    if not path.is_file():
        raise ValueError(f"image_file does not exist: {path}")
    return path.resolve()


def _positive_finite(name, value):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def main(args=None):
    import cv2
    from cv_bridge import CvBridge
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    class ImageReplayNode(Node):
        def __init__(self):
            super().__init__("image_replay_node")
            self.declare_parameter("image_file", "")
            self.declare_parameter("image_topic", "/smartcar/test/image")
            self.declare_parameter("frame_id", "smartcar_test_camera")
            self.declare_parameter("publish_rate_hz", 2.0)

            image_file = resolve_image_file(
                self.get_parameter("image_file").value)
            image_topic = str(
                self.get_parameter("image_topic").value).strip()
            frame_id = str(self.get_parameter("frame_id").value).strip()
            rate = _positive_finite(
                "publish_rate_hz",
                self.get_parameter("publish_rate_hz").value,
            )
            if not image_topic or not frame_id:
                raise ValueError("image_topic and frame_id must be nonempty")

            frame = cv2.imread(str(image_file), cv2.IMREAD_COLOR)
            if frame is None or frame.size == 0:
                raise ValueError(f"unable to decode image_file: {image_file}")
            self._message = CvBridge().cv2_to_imgmsg(
                frame, encoding="bgr8")
            self._message.header.frame_id = frame_id
            # zbar_ros requests a reliable image subscription. A reliable,
            # volatile replay publisher is also compatible with best-effort
            # image subscribers such as the vision service.
            image_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._publisher = self.create_publisher(
                Image, image_topic, image_qos)
            self._timer = self.create_timer(1.0 / rate, self._publish)
            self.get_logger().info(
                f"Replaying {image_file} on {image_topic} at {rate:.2f} Hz")

        def _publish(self):
            self._message.header.stamp = self.get_clock().now().to_msg()
            self._publisher.publish(self._message)

    rclpy.init(args=args)
    node = ImageReplayNode()
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
