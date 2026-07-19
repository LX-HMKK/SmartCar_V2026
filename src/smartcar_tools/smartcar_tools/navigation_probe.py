"""Fast, fail-closed CLI for the navigation test service sequence."""
import argparse
from dataclasses import dataclass
import json
import math
import sys


SUCCESS = 0
SEQUENCE_FAILED = 1
SERVICE_UNAVAILABLE = 2
TRANSPORT_ERROR = 3
INVALID_ARGUMENT = 4

START_STEPS = ("prepare", "arm", "start")
SERVICE_PREFIX = "/smartcar/test/navigation"


@dataclass(frozen=True)
class StepResult:
    success: bool
    message: str
    exit_code: int = SEQUENCE_FAILED


def run_start_sequence(call_step, fail_safe_stop):
    """Run the explicit three-step contract and lock down on any failure."""
    try:
        for step in START_STEPS:
            result = call_step(step)
            if not result.success:
                fail_safe_stop()
                return result.exit_code
    except BaseException:
        fail_safe_stop()
        raise
    return SUCCESS


def _positive(name, value):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Call the SmartCar navigation safety sequence in one ROS process."
        ),
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    start = subparsers.add_parser(
        "start",
        aliases=["run"],
        help="Wait for Nav2, then call prepare, arm, and start once",
    )
    start.add_argument("--ready-timeout-sec", type=float, default=60.0)
    stop = subparsers.add_parser(
        "stop",
        help="Latch emergency stop and cancel the active navigation goal",
    )
    for command in (start, stop):
        command.add_argument("--service-wait-sec", type=float, default=5.0)
        command.add_argument("--response-timeout-sec", type=float, default=15.0)
    return parser


def _run(options, ros_args):
    import rclpy
    from nav2_msgs.action import NavigateThroughPoses
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    service_wait_sec = _positive(
        "service-wait-sec", options.service_wait_sec)
    response_timeout_sec = _positive(
        "response-timeout-sec", options.response_timeout_sec)

    rclpy.init(args=ros_args)
    node = Node("navigation_probe")
    clients = {
        step: node.create_client(
            Trigger,
            f"{SERVICE_PREFIX}/{step}",
        )
        for step in (*START_STEPS, "stop")
    }

    def call_step(step, *, wait_sec=None, timeout_sec=None):
        client = clients[step]
        wait = service_wait_sec if wait_sec is None else float(wait_sec)
        timeout = (
            response_timeout_sec if timeout_sec is None else float(timeout_sec)
        )
        if not client.wait_for_service(timeout_sec=wait):
            result = StepResult(
                False,
                f"{step}_service_unavailable",
                SERVICE_UNAVAILABLE,
            )
        else:
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                node,
                future,
                timeout_sec=timeout,
            )
            if not future.done():
                try:
                    client.remove_pending_request(future)
                except (AttributeError, KeyError, RuntimeError):
                    pass
                result = StepResult(
                    False,
                    f"{step}_response_timeout",
                    TRANSPORT_ERROR,
                )
            else:
                try:
                    response = future.result()
                except Exception as error:
                    result = StepResult(
                        False,
                        f"{step}_transport_error:{type(error).__name__}",
                        TRANSPORT_ERROR,
                    )
                else:
                    if response is None:
                        result = StepResult(
                            False,
                            f"{step}_transport_error:empty_response",
                            TRANSPORT_ERROR,
                        )
                    else:
                        result = StepResult(
                            bool(response.success),
                            str(response.message),
                            SEQUENCE_FAILED,
                        )
        print(json.dumps({
            "step": step,
            "success": result.success,
            "message": result.message,
        }, ensure_ascii=False), flush=True)
        return result

    def fail_safe_stop():
        try:
            call_step(
                "stop",
                wait_sec=min(service_wait_sec, 1.0),
                timeout_sec=min(response_timeout_sec, 5.0),
            )
        except Exception as error:
            print(
                f"best-effort stop failed: {type(error).__name__}",
                file=sys.stderr,
                flush=True,
            )

    try:
        if options.command == "stop":
            result = call_step("stop")
            return SUCCESS if result.success else result.exit_code

        ready_timeout_sec = _positive(
            "ready-timeout-sec", options.ready_timeout_sec)
        action_client = ActionClient(
            node,
            NavigateThroughPoses,
            "/navigate_through_poses",
        )
        if not action_client.wait_for_server(timeout_sec=ready_timeout_sec):
            print(json.dumps({
                "step": "nav2_ready",
                "success": False,
                "message": "navigate_through_poses_server_unavailable",
            }), flush=True)
            fail_safe_stop()
            return SERVICE_UNAVAILABLE
        print(json.dumps({
            "step": "nav2_ready",
            "success": True,
            "message": "navigate_through_poses_server_ready",
        }), flush=True)
        return run_start_sequence(call_step, fail_safe_stop)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    import rclpy
    from rclpy.utilities import remove_ros_args

    raw_args = list(sys.argv if args is None else [sys.argv[0], *args])
    options = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    ros_args = None if args is None else list(args)
    try:
        return _run(options, ros_args)
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return INVALID_ARGUMENT
    except KeyboardInterrupt:
        return 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
