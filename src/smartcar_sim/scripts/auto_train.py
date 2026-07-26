#!/usr/bin/env python3
"""auto_train.py — 仿真自动导航测试与参数调优

嵌入 launch 内部运行 (Python rclpy)，避免外部 DDS 发现层问题。
每个航点发送 NavigateToPose 目标，监控里程计检测兜圈子。
"""

import json
import math
import time
import traceback
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry


@dataclass
class Waypoint:
    x: float; y: float; yaw: float; desc: str = ""


TRAIN_WAYPOINTS = [
    Waypoint(3.127, 0.977, 0.5236, "P->QR(forward)"),
    Waypoint(4.500, 1.500, -0.785, "QR->VLM(reverse)"),
]

# 每次测试的参数组合
TEST_PARAMS = [
    # (name, values_to_try)
    ("k", [0.3, 0.4, 0.5]),
    ("desired_linear_vel", [0.10, 0.15]),
    ("lookahead_dist", [0.4, 0.6]),
    ("min_turning_radius", [0.35, 0.45, 0.55]),
    ("xy_goal_tolerance", [0.15, 0.25]),
    ("yaw_goal_tolerance", [0.3, 0.5]),
]


class AutoTrain(Node):
    def __init__(self):
        super().__init__("auto_train")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        self._odom_sub = self.create_subscription(
            Odometry, "/odom_combined", self._odom_cb, qos)
        self._action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose")

        self._latest_odom = None
        self._history = []  # [(x, y, t), ...]
        self._results = []
        self._goal_active = False
        self._goal_start = (0.0, 0.0)
        self._goal_time = 0.0
        self._max_dist = 0.0
        self._dir_changes = 0
        self._last_progress_sign = 0
        self._last_logged_t = -1

        self.get_logger().info("AutoTrain 就绪")

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg
        t = time.time()
        self._history.append((msg.pose.pose.position.x,
                              msg.pose.pose.position.y, t))
        if len(self._history) > 600:
            self._history = self._history[-300:]

    def wait_for_server(self, timeout=90.0):
        self.get_logger().info(f"等待 navigate_to_pose action server ({timeout}s)...")
        ok = self._action_client.wait_for_server(timeout_sec=timeout)
        if ok:
            self.get_logger().info("Action server 就绪!")
        else:
            self.get_logger().error("Action server 超时!")
        return ok

    def yaw_to_quat(self, yaw: float):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def send_goal(self, wp: Waypoint, timeout=120.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "odom_combined"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        qz, qw = math.sin(wp.yaw / 2.0), math.cos(wp.yaw / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f"发送目标 {wp.desc}: ({wp.x:.2f},{wp.y:.2f})")
        self._goal_active = True
        self._goal_time = time.time()
        self._max_dist = 0.0
        self._dir_changes = 0
        self._last_progress_sign = 0
        if self._latest_odom:
            self._goal_start = (self._latest_odom.pose.pose.position.x,
                                self._latest_odom.pose.pose.position.y)

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

        # Spin until result or timeout
        start = time.time()
        while self._goal_active and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            self._monitor_progress(time.time() - self._goal_time)

        result = {
            "wp": wp.desc, "max_dist": round(self._max_dist, 3),
            "dir_changes": self._dir_changes,
            "outcome": "timeout" if self._goal_active else "done"
        }
        self._results.append(result)
        self._goal_active = False
        self.get_logger().info(f"结果 {wp.desc}: {json.dumps(result)}")
        return result

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("目标被拒绝!")
            self._goal_active = False
            return
        self.get_logger().info("目标已接受")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result()
        if result:
            self.get_logger().info(f"导航完成: {result.status}")
        self._goal_active = False

    def _monitor_progress(self, elapsed):
        if not self._latest_odom:
            return
        x = self._latest_odom.pose.pose.position.x
        y = self._latest_odom.pose.pose.position.y
        dist = math.sqrt((x - self._goal_start[0])**2 +
                         (y - self._goal_start[1])**2)
        if dist > self._max_dist:
            self._max_dist = dist

        # 检测方向变化（兜圈子）
        if self._latest_odom and elapsed > 2:
            dx = x - (self._history[-2][0] if len(self._history) > 1 else x)
            wx, wy = self._goal_start
            # toward goal direction
            gx = 3.127 if "P→QR" in str(self._results) else 4.5
            gy = 0.977 if "P→QR" in str(self._results) else 1.5
            progress = dx * (gx - x) + (y - (self._history[-2][1] if len(self._history) > 1 else y)) * (gy - y)
            sign = 1 if progress > 0 else (-1 if progress < 0 else 0)
            if sign != 0 and sign != self._last_progress_sign and self._last_progress_sign != 0:
                self._dir_changes += 1
            if sign != 0:
                self._last_progress_sign = sign

        # 节流日志：每 5 秒输出一次（用整数秒整除 + 去重）
        t_int = int(elapsed)
        if t_int > 0 and t_int % 5 == 0:
            last_key = getattr(self, "_last_logged_t", -1)
            if t_int != last_key:
                self._last_logged_t = t_int
                self.get_logger().info(
                    f"  t={elapsed:.0f}s dist={dist:.2f}m changes={self._dir_changes}")

    def run(self):
        self.get_logger().info("="*50)
        self.get_logger().info("AutoTrain 开始")
        self.get_logger().info("="*50)

        if not self.wait_for_server():
            return

        for wp in TRAIN_WAYPOINTS:
            self.get_logger().info(f"--- {wp.desc} ---")
            self.send_goal(wp, timeout=120.0)
            time.sleep(3)  # 目标间冷却

        self._save_results()

    def _save_results(self):
        path = "/tmp/auto_train_results.json"
        data = {"results": self._results, "timestamp": time.time()}
        with open(path, "w") as f:
            json.dump(data, f, indent=2, default=str)
        self.get_logger().info(f"结果已保存: {path}")

        # 打印摘要
        print("\n" + "="*50)
        print("AutoTrain 结果摘要")
        print("="*50)
        for r in self._results:
            circling = "⚠️ 兜圈子!" if r["dir_changes"] > 4 else "✅"
            stuck = "⚠️ 无进展!" if r["max_dist"] < 0.5 else "✅"
            print(f"  {r['wp']}: max={r['max_dist']}m "
                  f"reversals={r['dir_changes']} "
                  f"{circling} {stuck}")
        print("="*50)


def main():
    rclpy.init()
    node = AutoTrain()
    try:
        node.run()
    except Exception:
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
