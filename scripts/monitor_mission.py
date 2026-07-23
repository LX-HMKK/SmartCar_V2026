#!/usr/bin/env python3
"""
SmartCar 任务状态实时监控
用法: python3 monitor_mission.py [--once]
  --once   打印一次当前状态后退出
  (默认)   每秒刷新，Ctrl+C 退出
"""
import argparse
import signal
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quat(q) -> float:
    """从 geometry_msgs/Quaternion 提取 yaw"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    import math
    return math.atan2(siny, cosy)


class MissionMonitor(Node):
    def __init__(self):
        super().__init__("mission_monitor")

        self._task_state: str = "unknown"
        self._odom: Optional[Odometry] = None
        self._cmd_vel: Optional[TwistStamped] = None
        self._last_log: str = ""
        self._start_time: Optional[float] = None
        self._state_changes: list = []

        # 订阅
        self._sub_state = self.create_subscription(
            String, "/smartcar/task/state", self._on_state, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/odom_combined", self._on_odom, 10
        )
        self._sub_cmd = self.create_subscription(
            TwistStamped, "/cmd_vel_safe", self._on_cmd, 10
        )
        # 通过 rosout 抓 task_node 日志
        self._sub_log = self.create_subscription(
            String, "/rosout", self._on_rosout, 10
        )

    def _on_state(self, msg: String) -> None:
        old = self._task_state
        self._task_state = msg.data
        if old != msg.data:
            self._state_changes.append((time.time(), old, msg.data))
            if msg.data == "RUNNING" and self._start_time is None:
                self._start_time = time.time()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_cmd(self, msg: TwistStamped) -> None:
        self._cmd_vel = msg

    def _on_rosout(self, msg: String) -> None:
        if "task_node" in msg.data:
            self._last_log = msg.data

    def snapshot(self) -> dict:
        """返回当前状态快照"""
        info = {"state": self._task_state}

        if self._odom is not None:
            p = self._odom.pose.pose.position
            q = self._odom.pose.pose.orientation
            info["x"] = round(p.x, 3)
            info["y"] = round(p.y, 3)
            info["yaw_deg"] = round(yaw_from_quat(q) * 180.0 / 3.14159, 1)

        if self._cmd_vel is not None:
            info["v"] = round(self._cmd_vel.twist.linear.x, 3)
            info["w"] = round(self._cmd_vel.twist.angular.z, 3)
        else:
            info["v"] = 0.0
            info["w"] = 0.0

        if self._start_time is not None:
            elapsed = time.time() - self._start_time
            info["elapsed"] = f"{int(elapsed // 60)}m{elapsed % 60:.0f}s"
        else:
            info["elapsed"] = "--"

        return info

    def format(self) -> str:
        """格式化当前状态"""
        s = self.snapshot()
        lines = [
            "\033[2J\033[H",  # clear screen
            "╔══════════════════════════════════════════════════╗",
            "║        SmartCar 任务监控                         ║",
            "╠══════════════════════════════════════════════════╣",
            f"║  状态: {s['state']:<42s} ║",
            f"║  位置: x={s.get('x', '--'):>7s}  y={s.get('y', '--'):>7s}  θ={s.get('yaw_deg', '--'):>6s}°      ║",
            f"║  速度: v={s.get('v', '--'):>7s}  ω={s.get('w', '--'):>7s}                       ║",
            f"║  耗时: {s['elapsed']:<42s} ║",
            "╠══════════════════════════════════════════════════╣",
        ]
        # 最近的状态变化 (最多 5 条)
        recent = self._state_changes[-5:]
        if recent:
            lines.append("║  状态历史:                                         ║")
            for ts, old_s, new_s in recent:
                t_str = time.strftime("%H:%M:%S", time.localtime(ts))
                lines.append(f"║    {t_str}  {old_s} → {new_s:<30s} ║")
        lines.append("╚══════════════════════════════════════════════════╝")
        lines.append("  Ctrl+C 退出")
        return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="SmartCar 任务监控")
    parser.add_argument("--once", action="store_true", help="打印一次后退出")
    args = parser.parse_args()

    rclpy.init()
    monitor = MissionMonitor()

    def _shutdown(sig=None, frame=None):
        print("\n监控结束")
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        if args.once:
            # 等订阅收到数据
            deadline = time.time() + 5.0
            while time.time() < deadline:
                rclpy.spin_once(monitor, timeout_sec=0.1)
                if monitor._task_state != "unknown":
                    break
            print(monitor.format())
        else:
            while rclpy.ok():
                rclpy.spin_once(monitor, timeout_sec=0.5)
                print(monitor.format())
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
