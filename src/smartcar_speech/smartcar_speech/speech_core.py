"""ROS-independent speech processing and local player adapters."""
from dataclasses import dataclass
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading


@dataclass(frozen=True)
class SpeechRequest:
    request_id: str
    text: str


def utf8_size(value):
    try:
        return len(str(value).encode("utf-8"))
    except UnicodeEncodeError as error:
        raise ValueError("text must be valid UTF-8") from error


class SpeechProcessor:
    """Run one synthesis/playback job while publishing bounded status data."""

    def __init__(
        self,
        synthesizer,
        player,
        status_callback,
        max_text_bytes=1024,
    ):
        self._synthesizer = synthesizer
        self._player = player
        self._status_callback = status_callback
        self._max_text_bytes = int(max_text_bytes)
        if self._max_text_bytes <= 0:
            raise ValueError("max_text_bytes must be positive")
        self._cancelled = threading.Event()

    def process(self, request):
        request_id = str(request.request_id).strip()
        text = str(request.text).strip()
        if not request_id:
            raise ValueError("request_id must be nonempty")
        if self._cancelled.is_set():
            self._emit("cancelled", request_id, "shutdown")
            return False
        if not text:
            self._emit("failed", request_id, "empty_text")
            return False
        try:
            text_bytes = utf8_size(text)
        except ValueError:
            self._emit("failed", request_id, "invalid_text_encoding")
            return False
        if text_bytes > self._max_text_bytes:
            self._emit("failed", request_id, "text_too_long")
            return False

        self._emit("synthesizing", request_id)
        try:
            result = self._synthesizer.synthesize(text)
        except Exception as error:
            self._emit(
                "failed",
                request_id,
                self._failure_detail("synthesis", error),
            )
            return False

        if self._cancelled.is_set():
            self._emit("cancelled", request_id, "shutdown")
            return False
        self._emit("playing", request_id)
        try:
            self._player.play(result)
        except Exception as error:
            self._emit(
                "failed",
                request_id,
                self._failure_detail("playback", error),
            )
            return False
        self._emit("completed", request_id)
        return True

    def cancel(self):
        self._cancelled.set()
        cancel = getattr(self._player, "cancel", None)
        if callable(cancel):
            cancel()

    def _emit(self, state, request_id, detail=""):
        try:
            self._status_callback(str(state), str(request_id), str(detail))
        except Exception:
            # Status reporting must not duplicate synthesis or playback.
            pass

    @staticmethod
    def _failure_detail(stage, error):
        public_status = getattr(error, "public_status", "")
        if isinstance(public_status, str) and 0 < len(public_status) <= 128:
            return f"{stage}:{public_status}"
        return f"{stage}:{type(error).__name__}"


class PlaybackError(RuntimeError):
    def __init__(self, public_status):
        self.public_status = str(public_status)
        super().__init__(self.public_status)


class CommandPlayer:
    """Play synthesized bytes through a bounded argv command without a shell."""

    _SUFFIXES = {
        "mp3": ".mp3",
        "wav": ".wav",
        "ogg_opus": ".ogg",
    }

    def __init__(
        self,
        argv,
        runtime_dir="/tmp/smartcar_speech",
        timeout_sec=30.0,
        speaker_sink="",
        popen_factory=None,
    ):
        if isinstance(argv, (str, bytes)):
            raise ValueError("player argv must be a sequence, not a string")
        self._argv = tuple(str(value) for value in argv)
        if not self._argv or not self._argv[0].strip():
            raise ValueError("player argv must contain an executable")
        if not any("{audio_file}" in value for value in self._argv):
            raise ValueError("player argv must contain {audio_file}")
        self._runtime_dir = Path(str(runtime_dir)).expanduser()
        self._timeout_sec = float(timeout_sec)
        if not math.isfinite(self._timeout_sec) or self._timeout_sec <= 0.0:
            raise ValueError("player timeout must be finite and positive")
        self._speaker_sink = str(speaker_sink).strip()
        self._popen_factory = popen_factory or subprocess.Popen
        self._lock = threading.RLock()
        self._process = None
        self._cancelled = threading.Event()

    def play(self, result):
        audio = bytes(result.audio)
        if self._cancelled.is_set():
            raise PlaybackError("cancelled")
        if not audio:
            raise ValueError("audio must be nonempty")
        suffix = self._SUFFIXES.get(str(result.encoding).strip().lower())
        if suffix is None:
            raise ValueError("unsupported audio encoding")
        self._runtime_dir.mkdir(parents=True, exist_ok=True)
        path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb",
                prefix="speech-",
                suffix=suffix,
                dir=str(self._runtime_dir),
                delete=False,
            ) as audio_file:
                path = audio_file.name
                audio_file.write(audio)
            argv = [value.replace("{audio_file}", path) for value in self._argv]
            environment = os.environ.copy()
            if self._speaker_sink:
                environment["PULSE_SINK"] = self._speaker_sink
            process = self._popen_factory(
                argv,
                shell=False,
                env=environment,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            with self._lock:
                self._process = process
                cancelled = self._cancelled.is_set()
            if cancelled:
                self._stop_process(process)
                raise PlaybackError("cancelled")
            try:
                try:
                    returncode = process.wait(timeout=self._timeout_sec)
                except subprocess.TimeoutExpired as error:
                    self._stop_process(process)
                    raise PlaybackError("timeout") from error
            finally:
                with self._lock:
                    if self._process is process:
                        self._process = None
            if self._cancelled.is_set():
                raise PlaybackError("cancelled")
            if returncode != 0:
                raise PlaybackError(f"exit_{returncode}")
        finally:
            if path is not None:
                try:
                    Path(path).unlink()
                except FileNotFoundError:
                    pass

    def cancel(self):
        self._cancelled.set()
        with self._lock:
            process = self._process
        if process is not None:
            self._stop_process(process)

    @staticmethod
    def _stop_process(process):
        if process.poll() is not None:
            return
        try:
            process.terminate()
        except (OSError, ProcessLookupError):
            pass
        try:
            process.wait(timeout=0.25)
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            process.kill()
        except (OSError, ProcessLookupError):
            pass
        try:
            process.wait(timeout=0.25)
        except subprocess.TimeoutExpired:
            pass
