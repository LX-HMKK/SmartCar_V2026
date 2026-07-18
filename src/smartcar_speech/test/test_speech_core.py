from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_speech.speech_core import (  # noqa: E402
    CommandPlayer,
    PlaybackError,
    SpeechProcessor,
    SpeechRequest,
)
from smartcar_speech.volcengine_tts import SynthesisResult
from smartcar_speech.volcengine_tts import TtsError


class FakeSynthesizer:
    def __init__(self, result=None, error=None):
        self.result = result or SynthesisResult(
            audio=b"audio", encoding="mp3", request_id="backend-1")
        self.error = error
        self.calls = []

    def synthesize(self, text):
        self.calls.append(text)
        if self.error is not None:
            raise self.error
        return self.result


class FakePlayer:
    def __init__(self, error=None):
        self.error = error
        self.calls = []
        self.cancelled = False

    def play(self, result):
        self.calls.append(result)
        if self.error is not None:
            raise self.error

    def cancel(self):
        self.cancelled = True


class SpeechProcessorTests(unittest.TestCase):
    def test_success_is_serial_and_observable_without_ros(self):
        synth = FakeSynthesizer()
        player = FakePlayer()
        statuses = []
        processor = SpeechProcessor(
            synth, player, lambda *value: statuses.append(value))

        success = processor.process(SpeechRequest("job-1", "  hello  "))

        self.assertTrue(success)
        self.assertEqual(synth.calls, ["hello"])
        self.assertEqual(player.calls, [synth.result])
        self.assertEqual(
            [value[0] for value in statuses],
            ["synthesizing", "playing", "completed"],
        )
        self.assertTrue(all(value[1] == "job-1" for value in statuses))

    def test_synthesis_failure_never_invokes_player(self):
        synth = FakeSynthesizer(error=RuntimeError("offline"))
        player = FakePlayer()
        statuses = []
        processor = SpeechProcessor(
            synth, player, lambda *value: statuses.append(value))

        self.assertFalse(processor.process(SpeechRequest("job-2", "hello")))

        self.assertEqual(player.calls, [])
        self.assertEqual(statuses[-1], (
            "failed", "job-2", "synthesis:RuntimeError"))

    def test_safe_backend_status_is_exposed_without_exception_text(self):
        synth = FakeSynthesizer(error=TtsError("api_code:5501"))
        statuses = []
        processor = SpeechProcessor(
            synth,
            FakePlayer(),
            lambda *value: statuses.append(value),
        )

        self.assertFalse(processor.process(SpeechRequest("job-3", "hello")))

        self.assertEqual(
            statuses[-1],
            ("failed", "job-3", "synthesis:api_code:5501"),
        )

    def test_empty_or_oversized_text_never_calls_external_adapters(self):
        synth = FakeSynthesizer()
        player = FakePlayer()
        statuses = []
        processor = SpeechProcessor(
            synth,
            player,
            lambda *value: statuses.append(value),
            max_text_bytes=3,
        )

        self.assertFalse(processor.process(SpeechRequest("empty", "  ")))
        self.assertFalse(processor.process(SpeechRequest("long", "abcd")))

        self.assertEqual(synth.calls, [])
        self.assertEqual(player.calls, [])
        self.assertEqual(statuses, [
            ("failed", "empty", "empty_text"),
            ("failed", "long", "text_too_long"),
        ])

    def test_text_limit_is_measured_in_utf8_bytes(self):
        synth = FakeSynthesizer()
        statuses = []
        processor = SpeechProcessor(
            synth,
            FakePlayer(),
            lambda *value: statuses.append(value),
            max_text_bytes=1024,
        )

        self.assertTrue(processor.process(
            SpeechRequest("fits", "人" * 341)))
        self.assertFalse(processor.process(
            SpeechRequest("too-long", "人" * 342)))

        self.assertEqual(synth.calls, ["人" * 341])
        self.assertEqual(
            statuses[-1], ("failed", "too-long", "text_too_long"))

    def test_cancel_prevents_future_playback(self):
        player = FakePlayer()
        statuses = []
        processor = SpeechProcessor(
            FakeSynthesizer(),
            player,
            lambda *value: statuses.append(value),
        )
        processor.cancel()

        self.assertFalse(processor.process(
            SpeechRequest("cancelled", "hello")))
        self.assertTrue(player.cancelled)
        self.assertEqual(
            statuses, [("cancelled", "cancelled", "shutdown")])


class CommandPlayerTests(unittest.TestCase):
    def test_uses_argv_without_shell_and_removes_temporary_audio(self):
        calls = []

        class FakeProcess:
            def __init__(self):
                self.returncode = None
                self.wait_timeouts = []

            def wait(self, timeout):
                self.wait_timeouts.append(timeout)
                self.returncode = 0
                return self.returncode

            def poll(self):
                return self.returncode

            def terminate(self):
                self.returncode = -15

            def kill(self):
                self.returncode = -9

        process = FakeProcess()

        def fake_popen(argv, **options):
            audio_path = Path(argv[-1])
            calls.append((list(argv), dict(options), audio_path.read_bytes()))
            return process

        with tempfile.TemporaryDirectory() as runtime_dir:
            player = CommandPlayer(
                ["fake-player", "--quiet", "{audio_file}"],
                runtime_dir=runtime_dir,
                timeout_sec=4.0,
                speaker_sink="speaker-1",
                popen_factory=fake_popen,
            )
            player.play(SynthesisResult(
                audio=b"not-real-audio",
                encoding="mp3",
                request_id="request-1",
            ))
            remaining = list(Path(runtime_dir).iterdir())

        argv, options, audio = calls[0]
        self.assertEqual(argv[:2], ["fake-player", "--quiet"])
        self.assertEqual(audio, b"not-real-audio")
        self.assertFalse(options["shell"])
        self.assertEqual(options["env"]["PULSE_SINK"], "speaker-1")
        self.assertEqual(options["stdout"], subprocess.DEVNULL)
        self.assertEqual(options["stderr"], subprocess.DEVNULL)
        self.assertEqual(process.wait_timeouts, [4.0])
        self.assertEqual(remaining, [])

    def test_cancel_terminates_active_player_and_removes_audio(self):
        wait_started = threading.Event()
        process_stopped = threading.Event()
        errors = []

        class BlockingProcess:
            def __init__(self):
                self.returncode = None
                self.terminated = False

            def wait(self, timeout):
                wait_started.set()
                if not process_stopped.wait(timeout):
                    raise subprocess.TimeoutExpired("fake-player", timeout)
                return self.returncode

            def poll(self):
                return self.returncode

            def terminate(self):
                self.terminated = True
                self.returncode = -15
                process_stopped.set()

            def kill(self):
                self.returncode = -9
                process_stopped.set()

        process = BlockingProcess()
        with tempfile.TemporaryDirectory() as runtime_dir:
            player = CommandPlayer(
                ["fake-player", "{audio_file}"],
                runtime_dir=runtime_dir,
                timeout_sec=5.0,
                popen_factory=lambda *_args, **_kwargs: process,
            )

            def play():
                try:
                    player.play(SynthesisResult(
                        audio=b"audio",
                        encoding="mp3",
                        request_id="request-1",
                    ))
                except Exception as error:
                    errors.append(error)

            worker = threading.Thread(target=play)
            worker.start()
            self.assertTrue(wait_started.wait(1.0))
            player.cancel()
            worker.join(1.0)
            remaining = list(Path(runtime_dir).iterdir())

        self.assertFalse(worker.is_alive())
        self.assertTrue(process.terminated)
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], PlaybackError)
        self.assertEqual(errors[0].public_status, "cancelled")
        self.assertEqual(remaining, [])

    def test_rejects_shell_string_or_missing_audio_placeholder(self):
        with self.assertRaisesRegex(ValueError, "sequence"):
            CommandPlayer("player {audio_file}")
        with self.assertRaisesRegex(ValueError, "audio_file"):
            CommandPlayer(["player", "fixed.mp3"])


if __name__ == "__main__":
    unittest.main()
