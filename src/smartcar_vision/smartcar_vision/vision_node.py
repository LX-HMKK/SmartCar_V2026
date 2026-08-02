"""ROS 2 wrapper for QR and bounded local scene-description services."""
import math

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from smartcar_interfaces.srv import DescribeScene, ReadQr
from std_msgs.msg import String

from smartcar_vision.service_core import VisionServiceCore
from smartcar_vision.timed_sample import TimedSampleBuffer
from smartcar_vision.vlm_backend import make_backend


def _time_message_to_nanoseconds(message):
    seconds = int(message.sec)
    nanoseconds = int(message.nanosec)
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("not_before must be a nonnegative ROS time")
    return seconds * 1_000_000_000 + nanoseconds


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("barcode_topic", "/barcode")
        self.declare_parameter("vlm_backend_mode", "disabled")
        self.declare_parameter("vlm_command_argv", [""])
        self.declare_parameter("vlm_static_text", "")
        self.declare_parameter(
            "default_prompt", "请描述图中人物立牌的外观和动作。")
        self.declare_parameter("max_vlm_timeout_sec", 8.0)
        self.declare_parameter("runtime_dir", "/tmp/smartcar_vision")
        self.declare_parameter("jpeg_quality", 90)

        self._image_topic = str(self.get_parameter("image_topic").value)
        barcode_topic = str(self.get_parameter("barcode_topic").value)
        self._default_prompt = str(self.get_parameter("default_prompt").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        if not 1 <= self._jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")

        backend = make_backend(
            self.get_parameter("vlm_backend_mode").value,
            self.get_parameter("vlm_command_argv").value,
            self.get_parameter("vlm_static_text").value,
        )
        self._barcode_buffer = TimedSampleBuffer()
        self._image_buffer = TimedSampleBuffer()
        self._bridge = CvBridge()
        self._core = VisionServiceCore(
            barcode_buffer=self._barcode_buffer,
            image_buffer=self._image_buffer,
            backend=backend,
            jpeg_writer=self._write_jpeg,
            runtime_dir=self.get_parameter("runtime_dir").value,
            max_vlm_timeout_sec=self.get_parameter(
                "max_vlm_timeout_sec").value,
        )

        self._subscription_group = ReentrantCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()

        self._image_subscription = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            qos_profile_sensor_data,
            callback_group=self._subscription_group,
        )
        self._barcode_subscription = self.create_subscription(
            String,
            barcode_topic,
            self._on_barcode,
            10,
            callback_group=self._subscription_group,
        )
        self._read_qr_service = self.create_service(
            ReadQr,
            "/smartcar/vision/read_qr",
            self._on_read_qr,
            callback_group=self._service_group,
        )
        self._describe_service = self.create_service(
            DescribeScene,
            "/smartcar/vision/describe_scene",
            self._on_describe_scene,
            callback_group=self._service_group,
        )

        self.get_logger().info(
            f"Vision services ready on image topic {self._image_topic}")

    def _on_image(self, message):
        received_ns = self.get_clock().now().nanoseconds
        self._image_buffer.put(message, received_ns)

    def _on_barcode(self, message):
        received_ns = self.get_clock().now().nanoseconds
        self._barcode_buffer.put(message.data, received_ns)

    def _on_read_qr(self, request, response):
        try:
            not_before_ns = _time_message_to_nanoseconds(request.not_before)
            outcome = self._core.read_qr(
                not_before_ns, request.timeout_sec)
        except (TypeError, ValueError, OverflowError):
            response.success = False
            response.content = ""
            response.status = "invalid_request"
            return response

        response.success = outcome.success
        response.content = outcome.content
        response.status = outcome.status
        return response

    def _on_describe_scene(self, request, response):
        try:
            not_before_ns = _time_message_to_nanoseconds(request.not_before)
            timeout = float(request.timeout_sec)
            if not math.isfinite(timeout):
                raise ValueError("timeout must be finite")
            prompt = request.prompt or self._default_prompt
            outcome = self._core.describe_scene(
                not_before_ns, timeout, prompt)
        except (TypeError, ValueError, OverflowError):
            response.success = False
            response.fallback_used = False
            response.description = ""
            response.status = "invalid_request"
            return response

        response.success = outcome.success
        response.fallback_used = outcome.fallback_used
        response.description = outcome.description
        response.status = outcome.status
        return response

    def _write_jpeg(self, image_message, file_object):
        image = self._bridge.imgmsg_to_cv2(
            image_message, desired_encoding="bgr8")
        success, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
        )
        if not success:
            raise RuntimeError("OpenCV JPEG encoding failed")
        file_object.write(encoded.tobytes())


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
