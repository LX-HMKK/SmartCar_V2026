"""Record /PowerVoltage to a rotating log for offline voltage analysis."""
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


DEFAULT_LOG_PATH = "/tmp/voltage_history.log"
MAX_LINES = 100_000  # ~280 kB at ~50 bytes/line


class VoltageMonitor(Node):
    def __init__(self):
        super().__init__("voltage_monitor")
        self.declare_parameter("log_path", DEFAULT_LOG_PATH)
        self.declare_parameter("max_lines", MAX_LINES)

        self._log_path = str(self.get_parameter("log_path").value)
        self._max_lines = int(self.get_parameter("max_lines").value)
        self._line_count = 0

        self.create_subscription(
            Float32, "/PowerVoltage", self._on_voltage, 10
        )
        self.get_logger().info(
            f"Recording /PowerVoltage to {self._log_path}"
        )

    def _on_voltage(self, message):
        voltage = float(message.data)
        when = time.time()
        local = time.strftime(
            "%Y-%m-%dT%H:%M:%S", time.localtime(when)
        )
        line = f"{local}\t{when:.3f}\t{voltage:.3f}\n"

        try:
            with open(self._log_path, "a", encoding="utf-8") as stream:
                stream.write(line)
        except OSError:
            return  # silently skip if disk full or permissions wrong

        self._line_count += 1
        if self._line_count >= self._max_lines:
            self._rotate()

    def _rotate(self):
        try:
            os.replace(self._log_path, self._log_path + ".old")
        except OSError:
            pass
        self._line_count = 0
        self.get_logger().info(
            f"Voltage log rotated (>{self._max_lines} lines)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VoltageMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
