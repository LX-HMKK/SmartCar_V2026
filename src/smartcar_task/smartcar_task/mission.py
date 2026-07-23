"""ROS-independent mission state machine and port result contracts."""
from dataclasses import dataclass
from enum import Enum
import math
import threading


VLM_HARD_TIMEOUT_SEC = 8.0
VLM_FALLBACK_TEXT = "检测到人物立牌"
TERMINAL_STATES = frozenset({"COMPLETED", "STOPPED", "FAILED"})


class MissionState(str, Enum):
    IDLE = "IDLE"
    WAITING_FOR_SERVERS = "WAITING_FOR_SERVERS"
    NAVIGATING = "NAVIGATING"
    RUNNING_QR = "RUNNING_QR"
    RUNNING_VLM = "RUNNING_VLM"
    COMPLETED = "COMPLETED"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class OperationResult:
    success: bool
    status: str
    text: str = ""
    fallback_used: bool = False


def _finite(name, value, positive=False, nonnegative=False):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    if positive and result <= 0.0:
        raise ValueError(f"{name} must be positive")
    if nonnegative and result < 0.0:
        raise ValueError(f"{name} must be nonnegative")
    return result


def _count(name, value):
    if isinstance(value, bool) or int(value) != value or int(value) < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return int(value)


@dataclass(frozen=True)
class MissionConfig:
    server_wait_timeout_sec: float = 5.0
    navigation_retries: int = 1
    navigation_retry_delay_sec: float = 0.25
    qr_settle_sec: float = 2.0
    qr_timeout_sec: float = 3.0
    qr_retries: int = 1
    qr_retry_delay_sec: float = 0.25
    vlm_timeout_sec: float = 8.0
    vlm_prompt: str = "请描述图中人物立牌的外观和动作。"
    sleep_quantum_sec: float = 0.05
    vlm_fallback_text: str = VLM_FALLBACK_TEXT

    def __post_init__(self):
        _finite(
            "server_wait_timeout_sec",
            self.server_wait_timeout_sec,
            positive=True,
        )
        _count("navigation_retries", self.navigation_retries)
        _finite(
            "navigation_retry_delay_sec",
            self.navigation_retry_delay_sec,
            nonnegative=True,
        )
        _finite("qr_settle_sec", self.qr_settle_sec, nonnegative=True)
        _finite("qr_timeout_sec", self.qr_timeout_sec, positive=True)
        _count("qr_retries", self.qr_retries)
        _finite(
            "qr_retry_delay_sec",
            self.qr_retry_delay_sec,
            nonnegative=True,
        )
        vlm_timeout = _finite(
            "vlm_timeout_sec", self.vlm_timeout_sec, positive=True)
        if vlm_timeout > VLM_HARD_TIMEOUT_SEC:
            raise ValueError("vlm_timeout_sec cannot exceed 8 seconds")
        _finite("sleep_quantum_sec", self.sleep_quantum_sec, positive=True)
        if not str(self.vlm_prompt).strip():
            raise ValueError("vlm_prompt must be nonempty")
        if str(self.vlm_fallback_text) != VLM_FALLBACK_TEXT:
            raise ValueError("vlm_fallback_text is fixed by the competition contract")


class Mission:
    def __init__(
        self,
        navigator,
        vision,
        localization,
        clock,
        output,
        config=None,
    ):
        self._navigator = navigator
        self._vision = vision
        self._localization = localization
        self._clock = clock
        self._output = output
        self._config = config or MissionConfig()
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()
        self._state = MissionState.IDLE
        self._generation = 0
        self._reserved = False
        self._running = False
        self._resetting = False
        self._output.publish_state(self._state.value)

    @property
    def state(self):
        with self._lock:
            return self._state

    @property
    def running(self):
        with self._lock:
            return self._reserved or self._running or self._resetting

    def reserve_start(self):
        with self._lock:
            if (
                self._state is not MissionState.IDLE
                or self._reserved
                or self._running
                or self._resetting
                or self._navigator.is_active()
            ):
                return None
            self._generation += 1
            generation = self._generation
            self._reserved = True
            self._stop_requested.clear()
        self._set_state(MissionState.WAITING_FOR_SERVERS, generation)
        return generation

    def execute(self, waypoints):
        generation = self.reserve_start()
        if generation is None:
            return OperationResult(False, "mission_not_idle")
        return self.run_reserved(generation, waypoints)

    def run_reserved(self, generation, waypoints):
        with self._lock:
            if generation != self._generation or not self._reserved:
                return OperationResult(False, "mission_generation_invalid")
            self._reserved = False
            self._running = True
        try:
            return self._run(generation, tuple(waypoints))
        except Exception as error:
            if self._stop_requested.is_set():
                return self._finish_stopped(generation)
            return self._fail(
                generation,
                f"mission_exception:{type(error).__name__}",
            )
        finally:
            with self._lock:
                if generation == self._generation:
                    self._running = False

    def request_stop(self):
        with self._lock:
            if not (
                self._reserved
                or self._running
                or self._navigator.is_active()
            ):
                return False
            self._stop_requested.set()
        self._navigator.cancel()
        return True

    def reset(self):
        with self._lock:
            if (
                self._state.value not in TERMINAL_STATES
                or self._reserved
                or self._running
                or self._resetting
            ):
                return OperationResult(False, "reset_not_terminal")
            if self._navigator.is_active():
                return OperationResult(False, "navigation_not_stopped")
            self._resetting = True
            terminal_state = self._state
        try:
            result = self._localization.reset_origin()
        except Exception as error:
            result = OperationResult(
                False,
                f"reset_exception:{type(error).__name__}",
            )
        finally:
            with self._lock:
                self._resetting = False

        if not result.success:
            self._output.publish_text(f"定位复位失败: {result.status}")
            return result
        if self._navigator.is_active():
            self._set_state(terminal_state)
            return OperationResult(False, "navigation_reactivated")

        with self._lock:
            self._generation += 1
            self._stop_requested.clear()
        self._set_state(MissionState.IDLE)
        return OperationResult(True, "ok")

    def _run(self, generation, waypoints):
        if not waypoints:
            return self._fail(generation, "waypoints_empty")
        if self._stop_requested.is_set():
            return self._finish_stopped(generation)
        if not self._navigator.wait_ready(
            self._config.server_wait_timeout_sec
        ):
            return self._fail(generation, "navigation_server_unavailable")

        require_qr = any(item.task == "qr" for item in waypoints)
        require_vlm = any(item.task == "vlm" for item in waypoints)
        if (require_qr or require_vlm) and not self._vision.wait_ready(
            require_qr,
            require_vlm,
            self._config.server_wait_timeout_sec,
        ):
            return self._fail(generation, "vision_service_unavailable")
        if self._stop_requested.is_set():
            return self._finish_stopped(generation)

        for segment in self._navigation_segments(waypoints):
            navigation = self._navigate(generation, segment)
            if not navigation.success:
                return navigation
            endpoint_task = segment[-1].task
            if endpoint_task == "qr":
                task_result = self._run_qr(generation)
            elif endpoint_task == "vlm":
                task_result = self._run_vlm(generation)
            else:
                task_result = OperationResult(True, "ok")
            if not task_result.success:
                return task_result

        if self._stop_requested.is_set():
            return self._finish_stopped(generation)
        self._set_state(MissionState.COMPLETED, generation)
        return OperationResult(True, "mission_completed")

    @staticmethod
    def _navigation_segments(waypoints):
        segment = []
        for waypoint in waypoints:
            if not segment and waypoint.task == "start":
                continue  # car already at start, skip zero-length nav
            segment.append(waypoint)
            if waypoint.task in {"qr", "vlm", "return"}:
                yield tuple(segment)
                segment.clear()
        if segment:
            yield tuple(segment)

    def _navigate(self, generation, segment):
        self._set_state(MissionState.NAVIGATING, generation)
        attempts = self._config.navigation_retries + 1
        last_status = "navigation_failed"
        for attempt in range(attempts):
            if self._stop_requested.is_set():
                return self._finish_stopped(generation)
            result = self._navigator.navigate(segment)
            if self._stop_requested.is_set():
                return self._finish_stopped(generation)
            if result.success:
                return result
            last_status = result.status
            if self._navigator.is_active():
                return self._fail(
                    generation,
                    f"navigation_not_terminal:{last_status}",
                )
            if attempt + 1 < attempts and not self._interruptible_sleep(
                self._config.navigation_retry_delay_sec
            ):
                return self._finish_stopped(generation)
        return self._fail(generation, f"navigation_failed:{last_status}")

    def _run_qr(self, generation):
        self._set_state(MissionState.RUNNING_QR, generation)
        if not self._interruptible_sleep(self._config.qr_settle_sec):
            return self._finish_stopped(generation)

        attempts = self._config.qr_retries + 1
        last_status = "qr_failed"
        for attempt in range(attempts):
            not_before_ns = self._clock.now_ns()
            result = self._vision.read_qr(
                not_before_ns,
                self._config.qr_timeout_sec,
            )
            if self._stop_requested.is_set():
                return self._finish_stopped(generation)
            content = str(result.text).strip()
            if result.success and content:
                self._publish_value(content)
                return OperationResult(True, result.status, content)
            last_status = result.status
            if attempt + 1 < attempts and not self._interruptible_sleep(
                self._config.qr_retry_delay_sec
            ):
                return self._finish_stopped(generation)
        return self._fail(generation, f"qr_failed:{last_status}")

    def _run_vlm(self, generation):
        self._set_state(MissionState.RUNNING_VLM, generation)
        result = self._vision.describe_scene(
            self._clock.now_ns(),
            self._config.vlm_timeout_sec,
            self._config.vlm_prompt,
        )
        if self._stop_requested.is_set():
            return self._finish_stopped(generation)
        description = str(result.text).strip()
        if not result.success or not description:
            description = self._config.vlm_fallback_text
            result = OperationResult(
                True,
                result.status or "vlm_failed",
                description,
                True,
            )
        self._publish_value(description)
        return result

    def _interruptible_sleep(self, duration_sec):
        remaining = float(duration_sec)
        quantum = float(self._config.sleep_quantum_sec)
        while remaining > 0.0:
            if self._stop_requested.is_set():
                return False
            step = min(quantum, remaining)
            self._clock.sleep(step)
            remaining = max(0.0, remaining - step)
        return not self._stop_requested.is_set()

    def _finish_stopped(self, generation):
        if self._navigator.is_active():
            return self._fail(generation, "navigation_stop_unconfirmed")
        self._set_state(MissionState.STOPPED, generation)
        return OperationResult(False, "mission_stopped")

    def _fail(self, generation, status):
        self._set_state(MissionState.FAILED, generation)
        self._output.publish_text(f"任务失败: {status}")
        return OperationResult(False, status)

    def _publish_value(self, value):
        self._output.publish_text(value)
        self._output.publish_speech(value)

    def _set_state(self, state, generation=None):
        with self._lock:
            if generation is not None and generation != self._generation:
                return False
            if self._state is state:
                return True
            self._state = state
        self._output.publish_state(state.value)
        return True
