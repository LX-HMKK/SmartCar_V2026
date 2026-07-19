"""Tests for shell-free, timeout-bounded VLM command backends."""
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from smartcar_vision.vlm_backend import (  # noqa: E402
    CommandBackend,
    DisabledBackend,
    StaticBackend,
    make_backend,
)


class VlmBackendTests(unittest.TestCase):
    def test_command_expands_each_argument_without_shell_interpretation(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "shell-marker"
            prompt = f"hello; touch {marker}"
            backend = CommandBackend([
                sys.executable,
                "-c",
                "import json,sys; print(json.dumps(sys.argv[1:]))",
                "--image={image}",
                "--prompt={prompt}",
            ])

            result = backend.describe("image path.jpg", prompt, 2.0)

            self.assertTrue(result.ok, result.status)
            self.assertEqual(
                json.loads(result.text),
                ["--image=image path.jpg", f"--prompt={prompt}"],
            )
            self.assertFalse(marker.exists())

    def test_command_timeout_returns_error_promptly(self):
        backend = CommandBackend([
            sys.executable,
            "-c",
            "import time; time.sleep(5)",
        ])
        started = time.monotonic()
        result = backend.describe("unused.jpg", "unused", 0.05)

        self.assertFalse(result.ok)
        self.assertEqual(result.status, "backend_timeout")
        self.assertLess(time.monotonic() - started, 2.0)

    def test_nonzero_exit_and_empty_output_are_errors(self):
        failing = CommandBackend([
            sys.executable,
            "-c",
            "import sys; print('bad', file=sys.stderr); sys.exit(7)",
        ]).describe("unused.jpg", "unused", 2.0)
        empty = CommandBackend([
            sys.executable,
            "-c",
            "pass",
        ]).describe("unused.jpg", "unused", 2.0)

        self.assertFalse(failing.ok)
        self.assertTrue(failing.status.startswith("backend_exit_7"))
        self.assertFalse(empty.ok)
        self.assertEqual(empty.status, "backend_empty_output")

    def test_command_decodes_utf8_description_independent_of_locale(self):
        result = CommandBackend([
            sys.executable,
            "-X",
            "utf8",
            "-c",
            "print('人物正在挥手')",
        ]).describe("unused.jpg", "unused", 2.0)

        self.assertTrue(result.ok, result.status)
        self.assertEqual(result.text, "人物正在挥手")

    def test_static_and_disabled_backends(self):
        static = StaticBackend("bench description").describe(
            "unused.jpg", "unused", 1.0)
        disabled = DisabledBackend().describe("unused.jpg", "unused", 1.0)

        self.assertTrue(static.ok)
        self.assertEqual(static.text, "bench description")
        self.assertFalse(disabled.ok)
        self.assertEqual(disabled.status, "backend_disabled")

    def test_factory_and_validation(self):
        self.assertIsInstance(make_backend("static", [], "text"), StaticBackend)
        self.assertIsInstance(make_backend("disabled", [], ""), DisabledBackend)
        self.assertIsInstance(
            make_backend("command", [sys.executable, "-V"], ""),
            CommandBackend,
        )
        with self.assertRaises(ValueError):
            make_backend("unknown", [], "")
        with self.assertRaises(ValueError):
            CommandBackend([])
        for timeout in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(timeout=timeout):
                result = StaticBackend("text").describe(
                    "unused.jpg", "unused", timeout)
                self.assertFalse(result.ok)
                self.assertEqual(result.status, "backend_timeout")

    @unittest.skipUnless(os.name == "posix", "POSIX process-group check")
    def test_timeout_kills_child_process_group(self):
        with tempfile.TemporaryDirectory() as directory:
            pid_file = Path(directory) / "child.pid"
            child_code = (
                "import os,pathlib,sys,time; "
                "pathlib.Path(sys.argv[1]).write_text(str(os.getpid())); "
                "time.sleep(30)"
            )
            parent_code = (
                "import subprocess,sys,time; "
                "subprocess.Popen([sys.executable,'-c',sys.argv[2],sys.argv[1]]); "
                "time.sleep(30)"
            )
            backend = CommandBackend([
                sys.executable,
                "-c",
                parent_code,
                str(pid_file),
                child_code,
            ])

            result = backend.describe("unused.jpg", "unused", 0.4)
            self.assertEqual(result.status, "backend_timeout")
            self.assertTrue(pid_file.exists())
            child_pid = int(pid_file.read_text(encoding="utf-8"))

            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and _process_is_running(child_pid):
                time.sleep(0.02)
            self.assertFalse(_process_is_running(child_pid))

    @unittest.skipUnless(os.name == "posix", "POSIX process-group check")
    def test_timeout_kills_child_after_group_leader_exits(self):
        with tempfile.TemporaryDirectory() as directory:
            pid_file = Path(directory) / "child.pid"
            child_code = "import time; time.sleep(30)"
            parent_code = (
                "import pathlib,subprocess,sys; "
                "child=subprocess.Popen([sys.executable,'-c',sys.argv[2]]); "
                "pathlib.Path(sys.argv[1]).write_text(str(child.pid))"
            )
            backend = CommandBackend([
                sys.executable,
                "-c",
                parent_code,
                str(pid_file),
                child_code,
            ])

            started = time.monotonic()
            result = backend.describe("unused.jpg", "unused", 0.4)
            elapsed = time.monotonic() - started

            self.assertEqual(result.status, "backend_timeout")
            self.assertLess(elapsed, 1.0)
            self.assertTrue(pid_file.exists())
            child_pid = int(pid_file.read_text(encoding="utf-8"))
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and _process_is_running(child_pid):
                time.sleep(0.02)
            self.assertFalse(_process_is_running(child_pid))


def _process_is_running(pid):
    stat_file = Path(f"/proc/{pid}/stat")
    if not stat_file.exists():
        return False
    fields = stat_file.read_text(encoding="utf-8").split()
    return len(fields) >= 3 and fields[2] != "Z"


if __name__ == "__main__":
    unittest.main()
