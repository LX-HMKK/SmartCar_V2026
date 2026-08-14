"""HDMI UI for the current QR or VLM competition text output."""
import os
import sys
import threading


os.environ.setdefault("DISPLAY", ":0")

try:
    from PyQt5.QtCore import Qt, QObject, pyqtSignal
    from PyQt5.QtWidgets import (
        QApplication,
        QLabel,
        QMainWindow,
        QVBoxLayout,
        QWidget,
    )
except ImportError as error:  # Keep import-only tests usable without PyQt5.
    _QT_IMPORT_ERROR = error
    QApplication = None
else:
    _QT_IMPORT_ERROR = None


def normalize_output_text(value):
    text = str(value).strip()
    return text or "等待比赛输出"


if QApplication is not None:
    class OutputBridge(QObject):
        text_received = pyqtSignal(str)


    class CompetitionOutputWindow(QMainWindow):
        def __init__(self, bridge, window_title, initial_text):
            super().__init__()
            self.setWindowTitle(window_title)
            self.setMinimumSize(900, 450)
            self.resize(1280, 720)
            self.setStyleSheet("""
                QMainWindow { background: #eef1f4; color: #17212b; }
                QLabel#title { font-size: 38px; font-weight: 700; }
                QLabel#output {
                    background: #ffffff;
                    border: 1px solid #c5ced6;
                    font-size: 42px;
                    padding: 36px;
                }
            """)
            root = QWidget(self)
            layout = QVBoxLayout(root)
            layout.setContentsMargins(36, 36, 36, 36)
            layout.setSpacing(24)

            title = QLabel(window_title)
            title.setObjectName("title")
            title.setAlignment(Qt.AlignCenter)
            layout.addWidget(title)

            self.output_label = QLabel(normalize_output_text(initial_text))
            self.output_label.setObjectName("output")
            self.output_label.setWordWrap(True)
            self.output_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(self.output_label, 1)
            self.setCentralWidget(root)
            bridge.text_received.connect(self.set_text)

        def set_text(self, value):
            self.output_label.setText(normalize_output_text(value))
else:
    OutputBridge = None
    CompetitionOutputWindow = None


def _run(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String

    app = QApplication.instance() or QApplication([sys.argv[0]])
    app.setApplicationName("SmartCar Competition Output")
    bridge = OutputBridge()

    class OutputNode(Node):
        def __init__(self):
            super().__init__("competition_output_display")
            self.declare_parameter("output_topic", "/smartcar/output/text")
            self.declare_parameter("window_title", "比赛输出")
            self.declare_parameter("initial_text", "等待比赛输出")
            self.declare_parameter("fullscreen", True)
            self.output_topic = str(
                self.get_parameter("output_topic").value).strip()
            self.window_title = str(
                self.get_parameter("window_title").value).strip()
            self.initial_text = normalize_output_text(
                self.get_parameter("initial_text").value)
            self.fullscreen = bool(self.get_parameter("fullscreen").value)
            if not self.output_topic or not self.window_title:
                raise ValueError("output_topic and window_title must be nonempty")
            self._subscription = self.create_subscription(
                String,
                self.output_topic,
                lambda message: bridge.text_received.emit(message.data),
                10,
            )

    rclpy.init(args=args)
    node = OutputNode()
    window = CompetitionOutputWindow(
        bridge,
        node.window_title,
        node.initial_text,
    )
    if node.fullscreen:
        window.showFullScreen()
    else:
        window.show()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin,
        name="smartcar-competition-output-ros",
        daemon=True,
    )
    spin_thread.start()
    try:
        return app.exec_()
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    if QApplication is None:
        print(
            "PyQt5 is required for competition_output_display: "
            f"{_QT_IMPORT_ERROR}",
            file=sys.stderr,
        )
        return 2
    try:
        return _run(args)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
