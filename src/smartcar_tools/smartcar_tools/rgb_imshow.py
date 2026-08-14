"""Show the selected RGB ROS topic in an OpenCV imshow window."""
import threading


def normalize_window_title(value):
    title = str(value).strip()
    return title or "Aurora RGB"


def should_close(key_code):
    return key_code in (27, ord("q"), ord("Q"))


def main(args=None):
    import cv2
    from cv_bridge import CvBridge
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class RgbImshowNode(Node):
        def __init__(self):
            super().__init__("rgb_imshow")
            self.declare_parameter("image_topic", "/aurora/rgb/image_raw")
            self.declare_parameter("window_title", "Aurora RGB")
            self.declare_parameter("fullscreen", False)
            self.image_topic = str(
                self.get_parameter("image_topic").value).strip()
            self.window_title = normalize_window_title(
                self.get_parameter("window_title").value)
            self.fullscreen = bool(self.get_parameter("fullscreen").value)
            if not self.image_topic:
                raise ValueError("image_topic must be nonempty")
            self._bridge = CvBridge()
            self._lock = threading.Lock()
            self._latest_frame = None
            self._latest_sequence = 0
            self._conversion_warning_emitted = False
            self._subscription = self.create_subscription(
                Image,
                self.image_topic,
                self._on_image,
                qos_profile_sensor_data,
            )

        def _on_image(self, message):
            try:
                frame = self._bridge.imgmsg_to_cv2(
                    message, desired_encoding="bgr8")
            except Exception as error:
                if not self._conversion_warning_emitted:
                    self.get_logger().warning(
                        "Unable to show RGB frame: "
                        f"{type(error).__name__}")
                    self._conversion_warning_emitted = True
                return
            with self._lock:
                self._latest_frame = frame.copy()
                self._latest_sequence += 1

        def latest_frame(self, displayed_sequence):
            with self._lock:
                if self._latest_sequence == displayed_sequence:
                    return displayed_sequence, None
                return self._latest_sequence, self._latest_frame

    rclpy.init(args=args)
    node = RgbImshowNode()
    cv2.namedWindow(node.window_title, cv2.WINDOW_NORMAL)
    if node.fullscreen:
        cv2.setWindowProperty(
            node.window_title,
            cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN,
        )
    displayed_sequence = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            displayed_sequence, frame = node.latest_frame(displayed_sequence)
            if frame is not None:
                cv2.imshow(node.window_title, frame)
            if should_close(cv2.waitKey(1) & 0xFF):
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyWindow(node.window_title)
        except cv2.error:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
