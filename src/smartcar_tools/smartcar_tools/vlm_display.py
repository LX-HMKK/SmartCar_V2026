"""HDMI scene-description display backed by the SmartCar VLM service."""
from dataclasses import dataclass
import math
import os
import sys
import threading
import time


os.environ.setdefault("DISPLAY", ":0")

try:
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
    from PyQt5.QtWidgets import (
        QApplication,
        QLabel,
        QMainWindow,
        QPushButton,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError as error:  # Import remains optional for non-UI test hosts.
    _QT_IMPORT_ERROR = error
    QApplication = None
else:
    _QT_IMPORT_ERROR = None


@dataclass(frozen=True)
class DisplayResult:
    success: bool
    fallback_used: bool
    text: str
    status: str
    elapsed_sec: float


def result_kind(result):
    if not result.success or not str(result.text).strip():
        return "failed"
    if result.fallback_used:
        return "fallback"
    return "success"


def result_status_text(result):
    if result_kind(result) == "failed":
        return "生成失败"
    return "生成完成"


if QApplication is not None:
    class UiBridge(QObject):
        text_received = pyqtSignal(str)
        request_finished = pyqtSignal(object)
        auto_request = pyqtSignal()


    class VlmWindow(QMainWindow):
        def __init__(self, bridge, request_callback=None):
            super().__init__()
            self._request_callback = request_callback
            self._request_started_at = None

            self.setWindowTitle("SmartCar 图生文")
            self.setMinimumSize(900, 450)
            self.resize(1280, 720)
            self.setStyleSheet("""
                QMainWindow { background: #eef1f4; color: #17212b; }
                QLabel#title { font-size: 34px; font-weight: 700; }
                QLabel#status {
                    background: #e8edf1;
                    border-radius: 6px;
                    padding: 12px 16px;
                    font-size: 22px;
                    font-weight: 600;
                }
                QTextEdit {
                    background: #ffffff;
                    border: 1px solid #c5ced6;
                    border-radius: 6px;
                    padding: 18px;
                    font-size: 32px;
                }
                QPushButton {
                    background: #087f5b;
                    color: white;
                    border: 0;
                    border-radius: 6px;
                    min-height: 62px;
                    font-size: 26px;
                    font-weight: 700;
                    padding: 8px 22px;
                }
                QPushButton:disabled { background: #87949e; }
            """)

            root = QWidget(self)
            layout = QVBoxLayout(root)
            layout.setContentsMargins(36, 36, 36, 36)
            layout.setSpacing(24)

            title = QLabel("图生文结果")
            title.setObjectName("title")
            title.setAlignment(Qt.AlignCenter)
            layout.addWidget(title)

            self.status_label = QLabel("等待触发 · 0.0 s")
            self.status_label.setObjectName("status")
            self.status_label.setWordWrap(True)
            layout.addWidget(self.status_label)

            self.result_text = QTextEdit()
            self.result_text.setReadOnly(True)
            self.result_text.setAcceptRichText(False)
            self.result_text.setLineWrapMode(QTextEdit.WidgetWidth)
            self.result_text.setPlainText("等待图生文结果")
            layout.addWidget(self.result_text, 1)

            self.trigger_button = QPushButton("生成描述")
            self.trigger_button.clicked.connect(self._request)
            layout.addWidget(self.trigger_button)
            self.setCentralWidget(root)

            self._elapsed_timer = QTimer(self)
            self._elapsed_timer.setInterval(100)
            self._elapsed_timer.timeout.connect(self._update_elapsed)
            bridge.text_received.connect(self.set_external_text)
            bridge.request_finished.connect(self.finish_request)

        def set_request_callback(self, callback):
            self._request_callback = callback

        def _request(self):
            if self._request_callback is None:
                return
            self._request_started_at = time.monotonic()
            self.trigger_button.setEnabled(False)
            self.status_label.setText("处理中 · 0.0 s")
            self._elapsed_timer.start()
            self._request_callback()

        def _update_elapsed(self):
            if self._request_started_at is None:
                return
            elapsed = time.monotonic() - self._request_started_at
            self.status_label.setText(f"处理中 · {elapsed:.1f} s")

        def finish_request(self, result):
            self._elapsed_timer.stop()
            self._request_started_at = None
            self.trigger_button.setEnabled(True)
            self.status_label.setText(
                f"{result_status_text(result)} · {result.elapsed_sec:.1f} s")
            if str(result.text).strip():
                self.result_text.setPlainText(str(result.text).strip())
            elif result_kind(result) == "failed":
                self.result_text.setPlainText("未生成描述")

        def set_external_text(self, value):
            text = str(value).strip()
            if text:
                self.result_text.setPlainText(text)

else:
    UiBridge = None
    VlmWindow = None


def _positive_timeout(value):
    timeout = float(value)
    if not math.isfinite(timeout) or not 0.0 < timeout <= 8.0:
        raise ValueError("request_timeout_sec must be in (0, 8]")
    return timeout


def _run(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from smartcar_interfaces.srv import DescribeScene
    from std_msgs.msg import String

    app = QApplication.instance() or QApplication([sys.argv[0]])
    app.setApplicationName("SmartCar VLM Display")
    qt_bridge = UiBridge()

    class DisplayNode(Node):
        def __init__(self):
            super().__init__("vlm_display")
            self.declare_parameter(
                "image_topic", "/smartcar/test/image")
            self.declare_parameter(
                "output_topic", "/smartcar/output/text")
            self.declare_parameter(
                "service_name", "/smartcar/vision/describe_scene")
            self.declare_parameter(
                "prompt", "请描述当前画面中实际可见的场景和物体。")
            self.declare_parameter("request_timeout_sec", 8.0)
            self.declare_parameter("fullscreen", True)
            self.declare_parameter("auto_request", False)

            image_topic = str(
                self.get_parameter("image_topic").value).strip()
            output_topic = str(
                self.get_parameter("output_topic").value).strip()
            service_name = str(
                self.get_parameter("service_name").value).strip()
            if not image_topic or not output_topic or not service_name:
                raise ValueError("UI topics and service name must be nonempty")
            self.prompt = str(self.get_parameter("prompt").value)
            self.timeout_sec = _positive_timeout(
                self.get_parameter("request_timeout_sec").value)
            self.fullscreen = bool(
                self.get_parameter("fullscreen").value)
            self.auto_request = bool(
                self.get_parameter("auto_request").value)
            self._pending_lock = threading.RLock()
            self._pending = None
            self._pending_timer = None
            self._request_started_at = 0.0
            self._frame_warning_emitted = False
            self._auto_request_sent = False

            self._image_subscription = self.create_subscription(
                Image,
                image_topic,
                self._on_image,
                qos_profile_sensor_data,
            )
            self._text_subscription = self.create_subscription(
                String,
                output_topic,
                lambda message: qt_bridge.text_received.emit(message.data),
                10,
            )
            self._text_publisher = self.create_publisher(
                String, output_topic, 10)
            self._client = self.create_client(
                DescribeScene, service_name)

        def _on_image(self, message):
            if not message.data or message.width <= 0 or message.height <= 0:
                if not self._frame_warning_emitted:
                    self.get_logger().warning(
                        "Ignoring invalid camera frame before VLM request")
                    self._frame_warning_emitted = True
                return
            if (
                self.auto_request
                and not self._auto_request_sent
                and self._client.service_is_ready()
            ):
                self._auto_request_sent = True
                qt_bridge.auto_request.emit()

        def request_description(self):
            started = time.monotonic()
            if not self._client.service_is_ready():
                qt_bridge.request_finished.emit(DisplayResult(
                    False,
                    False,
                    "",
                    "service_unavailable",
                    time.monotonic() - started,
                ))
                return
            with self._pending_lock:
                if self._pending is not None:
                    qt_bridge.request_finished.emit(DisplayResult(
                        False,
                        False,
                        "",
                        "request_already_active",
                        0.0,
                    ))
                    return
                request = DescribeScene.Request()
                request.not_before = self.get_clock().now().to_msg()
                request.timeout_sec = self.timeout_sec
                request.prompt = self.prompt
                self._request_started_at = started
                try:
                    future = self._client.call_async(request)
                except Exception as error:
                    qt_bridge.request_finished.emit(DisplayResult(
                        False,
                        False,
                        "",
                        f"transport_error:{type(error).__name__}",
                        time.monotonic() - started,
                    ))
                    return
                timer = threading.Timer(
                    self.timeout_sec + 1.0,
                    self._request_timed_out,
                    args=(future,),
                )
                timer.daemon = True
                self._pending = future
                self._pending_timer = timer
                future.add_done_callback(self._request_done)
                if self._pending is future:
                    timer.start()

        def _request_timed_out(self, future):
            with self._pending_lock:
                if future is not self._pending:
                    return
                self._pending = None
                self._pending_timer = None
                started = self._request_started_at
            try:
                self._client.remove_pending_request(future)
            except (AttributeError, KeyError, RuntimeError):
                pass
            qt_bridge.request_finished.emit(DisplayResult(
                False,
                False,
                "",
                "transport_timeout",
                time.monotonic() - started,
            ))

        def _request_done(self, future):
            with self._pending_lock:
                if future is not self._pending:
                    return
                self._pending = None
                timer = self._pending_timer
                self._pending_timer = None
                started = self._request_started_at
            if timer is not None:
                timer.cancel()
            try:
                response = future.result()
            except Exception as error:
                result = DisplayResult(
                    False,
                    False,
                    "",
                    f"transport_error:{type(error).__name__}",
                    time.monotonic() - started,
                )
            else:
                result = DisplayResult(
                    bool(response.success),
                    bool(response.fallback_used),
                    str(response.description),
                    str(response.status),
                    time.monotonic() - started,
                )
            if result.success and result.text.strip():
                self._text_publisher.publish(String(data=result.text.strip()))
            qt_bridge.request_finished.emit(result)

        def stop(self):
            with self._pending_lock:
                timer = self._pending_timer
                self._pending_timer = None
                future = self._pending
                self._pending = None
            if timer is not None:
                timer.cancel()
            if future is not None:
                try:
                    self._client.remove_pending_request(future)
                except (AttributeError, KeyError, RuntimeError):
                    pass

    rclpy.init(args=args)
    node = DisplayNode()
    window = VlmWindow(qt_bridge, node.request_description)
    qt_bridge.auto_request.connect(window._request)
    if node.fullscreen:
        window.showFullScreen()
    else:
        window.show()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin,
        name="smartcar-vlm-display-ros",
        daemon=True,
    )
    spin_thread.start()
    try:
        return app.exec_()
    finally:
        node.stop()
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    if QApplication is None:
        print(
            f"PyQt5 is required for vlm_display: {_QT_IMPORT_ERROR}",
            file=sys.stderr,
        )
        return 2
    try:
        return _run(args)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
