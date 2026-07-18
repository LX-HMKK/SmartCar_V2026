"""Local scene-description backends with bounded subprocess execution."""
from dataclasses import dataclass
import math
import os
import signal
import subprocess
import time


def _positive_timeout(timeout_sec):
    try:
        timeout = float(timeout_sec)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(timeout) or timeout <= 0.0:
        return None
    return timeout


def _diagnostic(text, limit=200):
    normalized = " ".join((text or "").strip().split())
    return normalized[:limit]


@dataclass(frozen=True)
class VlmResult:
    ok: bool
    text: str
    status: str


class CommandBackend:
    """Invoke an argv-based local backend without a command shell."""

    def __init__(self, command_argv):
        if isinstance(command_argv, (str, bytes)):
            raise ValueError("command_argv must be a nonempty sequence of strings")
        try:
            argv = tuple(command_argv)
        except TypeError as error:
            raise ValueError(
                "command_argv must be a nonempty sequence of strings") from error
        if not argv or any(not isinstance(argument, str) for argument in argv):
            raise ValueError("command_argv must be a nonempty sequence of strings")
        if not argv[0]:
            raise ValueError("command executable must not be empty")
        self._command_argv = argv

    def describe(self, image_path, prompt, timeout_sec):
        timeout = _positive_timeout(timeout_sec)
        if timeout is None:
            return VlmResult(False, "", "backend_timeout")
        final_deadline = time.monotonic() + timeout
        cleanup_budget = min(
            0.25,
            max(0.01, timeout * 0.1),
            timeout * 0.5,
        )
        execution_deadline = final_deadline - cleanup_budget

        argv = [
            argument.replace("{image}", str(image_path)).replace(
                "{prompt}", str(prompt))
            for argument in self._command_argv
        ]
        popen_options = {
            "shell": False,
            "stdout": subprocess.PIPE,
            "stderr": subprocess.PIPE,
            "text": True,
            "encoding": "utf-8",
            "errors": "replace",
        }
        if os.name == "posix":
            popen_options["start_new_session"] = True
        elif os.name == "nt":
            popen_options["creationflags"] = getattr(
                subprocess, "CREATE_NEW_PROCESS_GROUP", 0)

        try:
            process = subprocess.Popen(argv, **popen_options)
        except (OSError, ValueError) as error:
            return VlmResult(
                False,
                "",
                f"backend_spawn_error:{type(error).__name__}",
            )

        remaining_execution = execution_deadline - time.monotonic()
        if remaining_execution <= 0.0:
            self._kill_process_tree(
                process, max(0.0, final_deadline - time.monotonic()))
            return VlmResult(False, "", "backend_timeout")

        try:
            stdout, stderr = process.communicate(timeout=remaining_execution)
        except subprocess.TimeoutExpired:
            self._kill_process_tree(
                process, max(0.0, final_deadline - time.monotonic()))
            return VlmResult(False, "", "backend_timeout")

        if process.returncode != 0:
            diagnostic = _diagnostic(stderr)
            suffix = f":{diagnostic}" if diagnostic else ""
            return VlmResult(
                False,
                "",
                f"backend_exit_{process.returncode}{suffix}",
            )

        output = (stdout or "").strip()
        if not output:
            return VlmResult(False, "", "backend_empty_output")
        return VlmResult(True, output, "ok")

    @staticmethod
    def _kill_process_tree(process, cleanup_timeout_sec):
        cleanup_timeout = max(0.0, float(cleanup_timeout_sec))
        cleanup_deadline = time.monotonic() + cleanup_timeout
        if os.name == "posix":
            # The leader may have exited while descendants still hold the
            # captured pipes. The process group can remain alive in that case.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except PermissionError:
                if process.poll() is None:
                    process.kill()
        elif os.name == "nt":
            remaining_cleanup = cleanup_deadline - time.monotonic()
            if remaining_cleanup > 0.0:
                try:
                    completed = subprocess.run(
                        [
                            "taskkill",
                            "/PID",
                            str(process.pid),
                            "/T",
                            "/F",
                        ],
                        shell=False,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=remaining_cleanup,
                        check=False,
                    )
                    if completed.returncode != 0 and process.poll() is None:
                        process.kill()
                except (OSError, subprocess.SubprocessError):
                    if process.poll() is None:
                        process.kill()
            elif process.poll() is None:
                process.kill()
        elif process.poll() is None:
            process.kill()

        remaining_cleanup = cleanup_deadline - time.monotonic()
        if remaining_cleanup > 0.0:
            try:
                process.communicate(timeout=remaining_cleanup)
            except subprocess.TimeoutExpired:
                pass

        if process.poll() is None:
            try:
                process.kill()
            except ProcessLookupError:
                pass
            remaining_cleanup = cleanup_deadline - time.monotonic()
            if remaining_cleanup > 0.0:
                try:
                    process.wait(timeout=remaining_cleanup)
                except subprocess.TimeoutExpired:
                    pass

        # Never perform an unbounded second communicate: a descendant may
        # have escaped cleanup while still holding these pipe descriptors.
        for stream in (process.stdout, process.stderr, process.stdin):
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass


class StaticBackend:
    def __init__(self, text):
        self._text = str(text)

    def describe(self, _image_path, _prompt, timeout_sec):
        if _positive_timeout(timeout_sec) is None:
            return VlmResult(False, "", "backend_timeout")
        output = self._text.strip()
        if not output:
            return VlmResult(False, "", "backend_empty_output")
        return VlmResult(True, output, "ok")


class DisabledBackend:
    def describe(self, _image_path, _prompt, timeout_sec):
        if _positive_timeout(timeout_sec) is None:
            return VlmResult(False, "", "backend_timeout")
        return VlmResult(False, "", "backend_disabled")


def make_backend(mode, command_argv, static_text):
    normalized = str(mode).strip().lower()
    if normalized == "command":
        return CommandBackend(command_argv)
    if normalized == "static":
        return StaticBackend(static_text)
    if normalized == "disabled":
        return DisabledBackend()
    raise ValueError("vlm_backend_mode must be command, static, or disabled")
