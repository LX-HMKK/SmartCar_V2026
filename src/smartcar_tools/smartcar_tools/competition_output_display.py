"""HDMI UI for the current QR or VLM competition text output."""
import os
import sys
import threading


os.environ.setdefault("DISPLAY", ":0")

try:
    from PyQt5.QtCore import QProcess, Qt, QObject, pyqtSignal
    from PyQt5.QtWidgets import (
        QApplication,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QTextEdit,
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


def competition_start_argv(command):
    """Build the exact UI-to-launcher command without a shell."""
    script = "" if command is None else str(command).strip()
    if not script:
        return ()
    return ("bash", script, "start", "--confirm")


if QApplication is not None:
    class OutputBridge(QObject):
        text_received = pyqtSignal(str)
        qr_received = pyqtSignal(str)
        c_zone_direction_received = pyqtSignal(str)
        vlm_received = pyqtSignal(str)
        state_received = pyqtSignal(str)


    class CompetitionOutputWindow(QMainWindow):
        def __init__(
            self,
            bridge,
            window_title,
            initial_text,
            initial_qr,
            initial_c_zone_direction,
            initial_vlm,
            initial_state,
            remote_start_enabled,
            remote_start_command,
        ):
            super().__init__()
            self.setWindowTitle(window_title)
            self.setMinimumSize(900, 450)
            self.resize(1280, 720)
            self.setStyleSheet("""
                QMainWindow { background: #eef1f4; color: #17212b; }
                QLabel#title { font-size: 34px; font-weight: 700; }
                QLabel#state { color: #4a5d6c; font-size: 20px; }
                QLabel#section { color: #3d5366; font-size: 22px; font-weight: 700; }
                QLabel#start_status { color: #4a5d6c; font-size: 18px; }
                QLabel#qr {
                    background: #ffffff;
                    border: 1px solid #c5ced6;
                    font-size: 44px;
                    font-weight: 700;
                    padding: 22px;
                }
                QLabel#c_zone_direction {
                    background: #ffffff;
                    border: 1px solid #c5ced6;
                    color: #165c85;
                    font-size: 28px;
                    font-weight: 700;
                    padding: 12px;
                }
                QTextEdit#vlm, QLabel#output {
                    background: #ffffff;
                    border: 1px solid #c5ced6;
                    font-size: 28px;
                    padding: 18px;
                }
                QPushButton#start {
                    background: #167d45;
                    border: 1px solid #126438;
                    color: #ffffff;
                    font-size: 28px;
                    font-weight: 700;
                    min-height: 62px;
                }
                QPushButton#start:disabled {
                    background: #aeb8b1;
                    border-color: #9ba69f;
                    color: #eff2f0;
                }
            """)
            root = QWidget(self)
            layout = QVBoxLayout(root)
            layout.setContentsMargins(28, 28, 28, 28)
            layout.setSpacing(12)
            self._start_command = (
                competition_start_argv(remote_start_command)
                if remote_start_enabled else ()
            )
            self._start_output = ""
            self._start_process = QProcess(self)
            self._start_process.setProcessChannelMode(QProcess.MergedChannels)
            self._start_process.readyReadStandardOutput.connect(
                self._handle_start_output)
            self._start_process.finished.connect(self._finish_start)
            self._start_process.errorOccurred.connect(self._handle_start_error)

            title = QLabel(window_title)
            title.setObjectName("title")
            title.setAlignment(Qt.AlignCenter)
            layout.addWidget(title)

            self.state_label = QLabel(normalize_output_text(initial_state))
            self.state_label.setObjectName("state")
            self.state_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(self.state_label)

            self.start_button = QPushButton("发车")
            self.start_button.setObjectName("start")
            self.start_button.setEnabled(bool(self._start_command))
            self.start_button.clicked.connect(self.request_start)
            layout.addWidget(self.start_button)

            self.start_status_label = QLabel(
                "急停锁存，等待发车"
                if self._start_command else "此界面未授权发车"
            )
            self.start_status_label.setObjectName("start_status")
            self.start_status_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(self.start_status_label)

            qr_title = QLabel("二维码结果")
            qr_title.setObjectName("section")
            layout.addWidget(qr_title)

            self.qr_label = QLabel(normalize_output_text(initial_qr))
            self.qr_label.setObjectName("qr")
            self.qr_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(self.qr_label)

            c_zone_title = QLabel("C区行驶方向")
            c_zone_title.setObjectName("section")
            layout.addWidget(c_zone_title)

            self.c_zone_direction_label = QLabel(
                normalize_output_text(initial_c_zone_direction))
            self.c_zone_direction_label.setObjectName("c_zone_direction")
            self.c_zone_direction_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(self.c_zone_direction_label)

            vlm_title = QLabel("诊疗区描述")
            vlm_title.setObjectName("section")
            layout.addWidget(vlm_title)

            self.vlm_output = QTextEdit()
            self.vlm_output.setObjectName("vlm")
            self.vlm_output.setReadOnly(True)
            self.vlm_output.setAcceptRichText(False)
            self.vlm_output.setLineWrapMode(QTextEdit.WidgetWidth)
            self.vlm_output.setPlainText(normalize_output_text(initial_vlm))
            layout.addWidget(self.vlm_output, 1)

            self.output_label = QLabel(normalize_output_text(initial_text))
            self.output_label.setObjectName("output")
            self.output_label.setWordWrap(True)
            self.output_label.setAlignment(Qt.AlignCenter)
            self.output_label.setMaximumHeight(96)
            layout.addWidget(self.output_label)
            self.setCentralWidget(root)
            bridge.text_received.connect(self.set_text)
            bridge.qr_received.connect(self.set_qr)
            bridge.c_zone_direction_received.connect(self.set_c_zone_direction)
            bridge.vlm_received.connect(self.set_vlm)
            bridge.state_received.connect(self.set_state)

        def set_text(self, value):
            self.output_label.setText(normalize_output_text(value))

        def set_qr(self, value):
            self.qr_label.setText(normalize_output_text(value))

        def set_c_zone_direction(self, value):
            self.c_zone_direction_label.setText(normalize_output_text(value))

        def set_vlm(self, value):
            self.vlm_output.setPlainText(normalize_output_text(value))

        def set_state(self, value):
            state = normalize_output_text(value)
            self.state_label.setText(state)
            if state in {
                "WAITING_FOR_SERVERS",
                "NAVIGATING",
                "RUNNING_QR",
                "RUNNING_VLM",
                "COMPLETED",
                "STOPPED",
                "FAILED",
            }:
                self.start_button.setEnabled(False)

        def request_start(self):
            if (
                not self._start_command
                or self._start_process.state() != QProcess.NotRunning
            ):
                return
            confirmation = QMessageBox(self)
            confirmation.setIcon(QMessageBox.Warning)
            confirmation.setWindowTitle("确认发车")
            confirmation.setText("确认远程发车？")
            confirmation.setInformativeText(
                "请确认车辆已人工放在 P 原点、车头朝 +X，物理急停可用。"
            )
            confirmation.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            confirmation.setDefaultButton(QMessageBox.No)
            if confirmation.exec_() != QMessageBox.Yes:
                return
            self._start_output = ""
            self.start_button.setEnabled(False)
            self.start_button.setText("发车中")
            self.start_status_label.setText("正在进行发车前检查")
            self._start_process.start(
                self._start_command[0], list(self._start_command[1:]))

        def _handle_start_output(self):
            output = bytes(
                self._start_process.readAllStandardOutput()).decode(
                    "utf-8", errors="replace").strip()
            if not output:
                return
            self._start_output = (self._start_output + "\n" + output)[-2000:]
            self.start_status_label.setText(output.splitlines()[-1])

        def _finish_start(self, exit_code, _exit_status):
            if exit_code == 0:
                self.start_button.setText("已发车")
                self.start_button.setEnabled(False)
                self.start_status_label.setText("比赛任务已触发")
                return
            self.start_button.setText("重新发车")
            self.start_button.setEnabled(bool(self._start_command))
            detail = self._start_output.splitlines()[-1:] or ["发车未执行"]
            self.start_status_label.setText(detail[0])

        def _handle_start_error(self, _error):
            if self._start_process.state() != QProcess.NotRunning:
                return
            self.start_button.setText("重新发车")
            self.start_button.setEnabled(bool(self._start_command))
            self.start_status_label.setText("无法启动发车流程")

        def closeEvent(self, event):
            if self._start_process.state() != QProcess.NotRunning:
                QMessageBox.warning(self, "发车进行中", "请等待发车流程结束。")
                event.ignore()
                return
            super().closeEvent(event)
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
            self.declare_parameter("qr_output_topic", "/smartcar/output/qr")
            self.declare_parameter(
                "c_zone_direction_topic", "/smartcar/output/c_zone_direction")
            self.declare_parameter("vlm_output_topic", "/smartcar/output/vlm")
            self.declare_parameter("task_state_topic", "/smartcar/task/state")
            self.declare_parameter("window_title", "比赛输出")
            self.declare_parameter("initial_text", "等待比赛输出")
            self.declare_parameter("initial_qr", "等待二维码")
            self.declare_parameter(
                "initial_c_zone_direction", "等待二维码后选择")
            self.declare_parameter("initial_vlm", "等待诊疗区描述")
            self.declare_parameter("initial_state", "系统就绪")
            self.declare_parameter("remote_start_enabled", False)
            self.declare_parameter("remote_start_command", "")
            self.declare_parameter("fullscreen", True)
            self.output_topic = str(
                self.get_parameter("output_topic").value).strip()
            self.qr_output_topic = str(
                self.get_parameter("qr_output_topic").value).strip()
            self.c_zone_direction_topic = str(
                self.get_parameter("c_zone_direction_topic").value).strip()
            self.vlm_output_topic = str(
                self.get_parameter("vlm_output_topic").value).strip()
            self.task_state_topic = str(
                self.get_parameter("task_state_topic").value).strip()
            self.window_title = str(
                self.get_parameter("window_title").value).strip()
            self.initial_text = normalize_output_text(
                self.get_parameter("initial_text").value)
            self.initial_qr = normalize_output_text(
                self.get_parameter("initial_qr").value)
            self.initial_c_zone_direction = normalize_output_text(
                self.get_parameter("initial_c_zone_direction").value)
            self.initial_vlm = normalize_output_text(
                self.get_parameter("initial_vlm").value)
            self.initial_state = normalize_output_text(
                self.get_parameter("initial_state").value)
            self.remote_start_enabled = bool(
                self.get_parameter("remote_start_enabled").value)
            self.remote_start_command = str(
                self.get_parameter("remote_start_command").value).strip()
            self.fullscreen = bool(self.get_parameter("fullscreen").value)
            if not all((
                self.output_topic,
                self.qr_output_topic,
                self.c_zone_direction_topic,
                self.vlm_output_topic,
                self.task_state_topic,
                self.window_title,
            )):
                raise ValueError("competition display topics and title must be nonempty")
            if self.remote_start_enabled and not self.remote_start_command:
                raise ValueError(
                    "remote_start_command is required when remote start is enabled")
            self._subscription = self.create_subscription(
                String,
                self.output_topic,
                lambda message: bridge.text_received.emit(message.data),
                10,
            )
            self._qr_subscription = self.create_subscription(
                String,
                self.qr_output_topic,
                lambda message: bridge.qr_received.emit(message.data),
                10,
            )
            self._c_zone_direction_subscription = self.create_subscription(
                String,
                self.c_zone_direction_topic,
                lambda message: bridge.c_zone_direction_received.emit(
                    message.data),
                10,
            )
            self._vlm_subscription = self.create_subscription(
                String,
                self.vlm_output_topic,
                lambda message: bridge.vlm_received.emit(message.data),
                10,
            )
            self._state_subscription = self.create_subscription(
                String,
                self.task_state_topic,
                lambda message: bridge.state_received.emit(message.data),
                10,
            )

    rclpy.init(args=args)
    node = OutputNode()
    window = CompetitionOutputWindow(
        bridge,
        node.window_title,
        node.initial_text,
        node.initial_qr,
        node.initial_c_zone_direction,
        node.initial_vlm,
        node.initial_state,
        node.remote_start_enabled,
        node.remote_start_command,
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
