"""Command-line client for the bounded SmartCar QR service."""
import argparse
import json
import math
import sys
import time


SUCCESS = 0
QR_NOT_FOUND = 1
SERVICE_UNAVAILABLE = 2
TRANSPORT_ERROR = 3
INVALID_ARGUMENT = 4


def response_exit_code(success, status):
    if bool(success) and str(status) == "ok":
        return SUCCESS
    return QR_NOT_FOUND


def competition_output_text(success, content, status):
    if not bool(success) or str(status) != "ok":
        return "未识别"
    value = str(content).strip()
    return value or "未识别"


def build_parser():
    parser = argparse.ArgumentParser(
        description="Read fresh QR values through /smartcar/vision/read_qr.",
    )
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--service-wait-sec", type=float, default=5.0)
    parser.add_argument("--response-grace-sec", type=float, default=1.0)
    parser.add_argument("--service", default="/smartcar/vision/read_qr")
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="Keep requesting fresh QR values",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help=(
            "Stop continuous mode after this many requests; "
            "zero is unlimited"
        ),
    )
    parser.add_argument("--interval-sec", type=float, default=0.25)
    parser.add_argument(
        "--output-topic",
        default="",
        help="Publish competition text to this topic after every request",
    )
    return parser


def _finite(name, value, allow_zero=False):
    result = float(value)
    valid = result >= 0.0 if allow_zero else result > 0.0
    if not math.isfinite(result) or not valid:
        qualifier = "nonnegative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return result


def _run(options, ros_args):
    import rclpy
    from rclpy.node import Node
    from smartcar_interfaces.srv import ReadQr
    from std_msgs.msg import String

    timeout_sec = _finite("timeout-sec", options.timeout_sec)
    wait_sec = _finite("service-wait-sec", options.service_wait_sec)
    grace_sec = _finite(
        "response-grace-sec", options.response_grace_sec, allow_zero=True)
    interval_sec = _finite(
        "interval-sec", options.interval_sec, allow_zero=True)
    if options.count < 0:
        raise ValueError("count must be nonnegative")
    output_topic = str(options.output_topic).strip()

    rclpy.init(args=ros_args)
    node = Node("qr_probe")
    client = node.create_client(ReadQr, options.service)
    output_publisher = (
        node.create_publisher(String, output_topic, 10)
        if output_topic else None
    )
    try:
        if not client.wait_for_service(timeout_sec=wait_sec):
            print("QR service unavailable", file=sys.stderr)
            return SERVICE_UNAVAILABLE

        attempts = 0
        final_code = SUCCESS
        while rclpy.ok():
            request = ReadQr.Request()
            request.not_before = node.get_clock().now().to_msg()
            request.timeout_sec = timeout_sec
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                node,
                future,
                timeout_sec=timeout_sec + grace_sec,
            )
            if not future.done():
                try:
                    client.remove_pending_request(future)
                except (AttributeError, KeyError, RuntimeError):
                    pass
                payload = {
                    "success": False,
                    "content": "",
                    "status": "transport_timeout",
                }
                code = TRANSPORT_ERROR
            else:
                try:
                    response = future.result()
                except Exception as error:
                    payload = {
                        "success": False,
                        "content": "",
                        "status": f"transport_error:{type(error).__name__}",
                    }
                    code = TRANSPORT_ERROR
                else:
                    if response is None:
                        payload = {
                            "success": False,
                            "content": "",
                            "status": "transport_error:empty_response",
                        }
                        code = TRANSPORT_ERROR
                    else:
                        payload = {
                            "success": bool(response.success),
                            "content": str(response.content),
                            "status": str(response.status),
                        }
                        code = response_exit_code(
                            response.success, response.status)
            print(json.dumps(payload, ensure_ascii=False), flush=True)
            if output_publisher is not None:
                output_publisher.publish(String(data=competition_output_text(
                    payload["success"],
                    payload["content"],
                    payload["status"],
                )))
            final_code = max(final_code, code)
            attempts += 1
            if not options.continuous:
                break
            if options.count and attempts >= options.count:
                break
            if interval_sec:
                time.sleep(interval_sec)
        return final_code
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
