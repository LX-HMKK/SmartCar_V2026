#!/usr/bin/env python3
"""仿真任务状态监控 —— 航点进度、当前动作、耗时，每秒刷新"""

import time
import sys
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class MissionMonitor(Node):
    def __init__(self):
        super().__init__("mission_monitor")
        self.start_time = time.time()
        self.current_waypoint = "N/A"
        self.current_action = "N/A"
        self.direction = "N/A"
        self.nav_status = "idle"

        # Subscribe to relevant topics via timers
        self.create_timer(1.0, self.tick)

        self.prev_status = None

    def tick(self):
        elapsed = time.time() - self.start_time
        # Query current navigation status from controller_server action
        status_line = (
            f"[{elapsed:6.1f}s] "
            f"wp: {self.current_waypoint:20s} "
            f"dir: {self.direction:7s} "
            f"nav: {self.nav_status:10s} "
            f"action: {self.current_action}"
        )

        if status_line != self.prev_status:
            self.get_logger().info(status_line)
            self.prev_status = status_line
        else:
            # Same status, show time only
            sys.stdout.write(f"\r[{elapsed:6.1f}s] {self.current_waypoint} | {self.direction} | {self.nav_status}     ")
            sys.stdout.flush()


def main():
    rclpy.init()
    monitor = MissionMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nMonitor stopped")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
