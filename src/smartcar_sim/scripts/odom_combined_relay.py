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

    def callback(self, msg: Odometry):
        now = self.get_clock().now().to_msg()
        # 重写 frame_id
        msg.header.frame_id = "odom_combined"
        msg.header.stamp = now
        msg.child_frame_id = "base_footprint"
        self.pub.publish(msg)

        # 发布 odom_combined → base_footprint TF（使用当前 sim time）
        t = TransformStamped()
        t.header.stamp = now
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
