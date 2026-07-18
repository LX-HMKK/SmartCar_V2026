"""ROS 2 subscriber that synthesizes SmartCar speech output off-executor."""
import json
import math
import os
import queue
import threading
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from smartcar_speech.speech_core import (
    CommandPlayer,
    SpeechProcessor,
    SpeechRequest,
    utf8_size,
)
from smartcar_speech.volcengine_tts import VolcengineV1TtsClient


class SpeechNode(Node):
    def __init__(self):
        super().__init__("speech_node")
        self.declare_parameter("enabled", False)
        self.declare_parameter("input_topic", "/smartcar/output/speech")
        self.declare_parameter("status_topic", "/smartcar/speech/status")
        self.declare_parameter(
            "api_url", "https://openspeech.bytedance.com/api/v1/tts")
        self.declare_parameter("app_id_env", "VOLCENGINE_TTS_APP_ID")
        self.declare_parameter(
            "access_token_env", "VOLCENGINE_TTS_ACCESS_TOKEN")
        self.declare_parameter("cluster", "volcano_tts")
        self.declare_parameter(
            "voice_type", "zh_male_M392_conversation_wvae_bigtts")
        self.declare_parameter("user_id", "smartcar_rdk_x5")
        self.declare_parameter("encoding", "mp3")
        self.declare_parameter("speed_ratio", 1.0)
        self.declare_parameter("request_timeout_sec", 10.0)
        self.declare_parameter("max_response_bytes", 8 * 1024 * 1024)
        self.declare_parameter("max_text_bytes", 1024)
        self.declare_parameter("queue_capacity", 4)
        self.declare_parameter("runtime_dir", "/tmp/smartcar_speech")
        self.declare_parameter(
            "player_argv",
            [
                "ffplay",
                "-nodisp",
                "-autoexit",
                "-loglevel",
                "quiet",
                "{audio_file}",
            ],
        )
        self.declare_parameter("player_timeout_sec", 30.0)
        self.declare_parameter("speaker_sink", "")
        self.declare_parameter("shutdown_timeout_sec", 2.0)

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_topic = str(self.get_parameter("status_topic").value).strip()
        input_topic = str(self.get_parameter("input_topic").value).strip()
        if not status_topic or not input_topic:
            raise ValueError("input_topic and status_topic must be nonempty")
        self._status_publisher = self.create_publisher(
            String, status_topic, status_qos)
        self._subscription = self.create_subscription(
            String, input_topic, self._on_speech, 10)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._shutdown_timeout_sec = float(
            self.get_parameter("shutdown_timeout_sec").value)
        if (
            not math.isfinite(self._shutdown_timeout_sec)
            or self._shutdown_timeout_sec < 0.0
        ):
            raise ValueError("shutdown_timeout_sec must be finite and nonnegative")
        self._max_text_bytes = int(
            self.get_parameter("max_text_bytes").value)
        if self._max_text_bytes <= 0:
            raise ValueError("max_text_bytes must be positive")
        capacity = int(self.get_parameter("queue_capacity").value)
        if capacity <= 0:
            raise ValueError("queue_capacity must be positive")
        self._queue = queue.Queue(maxsize=capacity)
        self._stop_event = threading.Event()
        self._worker = None
        self._processor = None

        if not self._enabled:
            self._publish_status("disabled")
            self.get_logger().info("Speech synthesis is disabled")
            return

        app_id_env = str(self.get_parameter("app_id_env").value).strip()
        token_env = str(self.get_parameter("access_token_env").value).strip()
        if not app_id_env or not token_env:
            raise ValueError("credential environment variable names must be nonempty")
        app_id = os.environ.get(app_id_env, "").strip()
        access_token = os.environ.get(token_env, "").strip()
        if not app_id or not access_token:
            self._publish_status("unconfigured", detail="credentials_missing")
            self.get_logger().error(
                "Speech is enabled but credential environment variables are missing")
            return

        synthesizer = VolcengineV1TtsClient(
            app_id=app_id,
            access_token=access_token,
            api_url=self.get_parameter("api_url").value,
            cluster=self.get_parameter("cluster").value,
            voice_type=self.get_parameter("voice_type").value,
            user_id=self.get_parameter("user_id").value,
            encoding=self.get_parameter("encoding").value,
            speed_ratio=self.get_parameter("speed_ratio").value,
            timeout_sec=self.get_parameter("request_timeout_sec").value,
            max_response_bytes=self.get_parameter("max_response_bytes").value,
        )
        player = CommandPlayer(
            argv=self.get_parameter("player_argv").value,
            runtime_dir=self.get_parameter("runtime_dir").value,
            timeout_sec=self.get_parameter("player_timeout_sec").value,
            speaker_sink=self.get_parameter("speaker_sink").value,
        )
        self._processor = SpeechProcessor(
            synthesizer=synthesizer,
            player=player,
            status_callback=self._publish_status,
            max_text_bytes=self._max_text_bytes,
        )
        self._worker = threading.Thread(
            target=self._worker_main,
            name="smartcar-speech",
            daemon=True,
        )
        self._worker.start()
        self._publish_status("ready")
        self.get_logger().info("Speech synthesis is ready")

    def _on_speech(self, message):
        request_id = str(uuid.uuid4())
        text = str(message.data).strip()
        if not text:
            self._publish_status("ignored", request_id, "empty_text")
            return
        try:
            text_bytes = utf8_size(text)
        except ValueError:
            self._publish_status(
                "failed", request_id, "invalid_text_encoding")
            return
        if text_bytes > self._max_text_bytes:
            self._publish_status("failed", request_id, "text_too_long")
            return
        if not self._enabled:
            self._publish_status("disabled", request_id)
            return
        if self._processor is None:
            self._publish_status("unconfigured", request_id)
            return
        request = SpeechRequest(request_id=request_id, text=text)
        try:
            self._queue.put_nowait(request)
        except queue.Full:
            self._publish_status("dropped", request_id, "queue_full")
            return
        self._publish_status("queued", request_id)

    def _worker_main(self):
        while not self._stop_event.is_set():
            try:
                request = self._queue.get(timeout=0.10)
            except queue.Empty:
                continue
            try:
                if self._stop_event.is_set():
                    self._publish_status(
                        "cancelled", request.request_id, "shutdown")
                    continue
                self._processor.process(request)
            finally:
                self._queue.task_done()

    def _publish_status(self, state, request_id="", detail=""):
        payload = json.dumps(
            {
                "state": str(state),
                "request_id": str(request_id),
                "detail": str(detail),
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self._status_publisher.publish(String(data=payload))
        if state == "failed":
            self.get_logger().error(
                f"Speech request {request_id} failed ({detail})")
        elif state == "dropped":
            self.get_logger().warning(
                f"Speech request {request_id} dropped ({detail})")

    def stop(self):
        self._stop_event.set()
        if self._processor is not None:
            self._processor.cancel()
        while True:
            try:
                request = self._queue.get_nowait()
            except queue.Empty:
                break
            self._publish_status(
                "cancelled", request.request_id, "shutdown")
            self._queue.task_done()
        if self._worker is not None:
            self._worker.join(timeout=self._shutdown_timeout_sec)
            if self._worker.is_alive():
                self.get_logger().warning(
                    "Speech worker did not stop before the shutdown deadline")


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
