#!/usr/bin/env python3
"""odom_combined_relay.py — 将 /odom_clean 直接转为 /odom_combined

绕过 EKF，直接提供 Nav2 所需的 odom_combined 坐标系。
发布 odom_combined → base_footprint TF（仿真用，无漂移）。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomCombinedRelay(Node):
    def __init__(self):
        super().__init__("odom_combined_relay")
        self.sub = self.create_subscription(
            Odometry, "/odom_clean", self.callback, 10
        )
        self.pub = self.create_publisher(Odometry, "/odom_combined", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            "odom_combined_relay: /odom_clean → /odom_combined + TF"
        )
        # ── 启动期 TF 兜底 ──
        # 修复原因：原 1Hz 兜底太慢。Gazebo 启动到首次 odom 发布可能需要
        # 2-5s（加载世界、生成模型、启动物理引擎）。
        # 这期间 RViz 需要 odom_combined frame 存在才能正常渲染 Grid/TF。
        # 策略：0.1s 首发 → 5Hz 高频兜底 5s → 之后降为 1Hz 长期兜底。
        # 一旦真实 odom 到达，所有兜底定时器立即取消。
        self._fallback_count = 0
        self._init_timer = self.create_timer(0.1, self._publish_init_tf)

    def _publish_init_tf(self):
        """Publish identity TF so odom_combined frame exists before first odom."""
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom_combined"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # Cancel one-shot init timer
        self._init_timer.cancel()

        # Cancel previous fallback timer to avoid timer accumulation
        # (each call creates a new periodic timer; must stop the old one)
        if hasattr(self, "_fallback_timer") and self._fallback_timer:
            self._fallback_timer.cancel()

        # First 5s: publish fallback at 5Hz (0.2s interval)
        # After 5s: remain at 1Hz as long-term safety net
        self._fallback_count += 1
        if self._fallback_count <= 25:  # 5s at 5Hz
            self._fallback_timer = self.create_timer(0.2, self._publish_init_tf)
        else:
            self._fallback_timer = self.create_timer(1.0, self._publish_init_tf)

    def callback(self, msg: Odometry):
        # Cancel fallback timer once real odom arrives
        if hasattr(self, "_fallback_timer") and self._fallback_timer:
            self._fallback_timer.cancel()
            self._fallback_timer = None

        # 使用 odom 消息的原始时间戳，而非 now()
        # 这样 TF 的时间戳与里程计数据一致，避免时序断裂
        stamp = msg.header.stamp

        # 重写 frame_id
        msg.header.frame_id = "odom_combined"
        msg.header.stamp = stamp
        msg.child_frame_id = "base_footprint"
        self.pub.publish(msg)

        # 发布 odom_combined → base_footprint TF（使用 odom 原始时间戳）
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom_combined"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomCombinedRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
