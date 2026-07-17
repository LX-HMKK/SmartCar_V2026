#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile

from ackermann_math import ackermann_command


class CmdVel2AckermannDriveNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_drive')
        self.wheelbase = self.declare_parameter('wheelbase', 0.189).value
        self.max_steering_angle = self.declare_parameter(
            'max_steering_angle', 0.45
        ).value
        input_topic = self.declare_parameter(
            'input_topic', '/cmd_vel_safe'
        ).value
        output_topic = self.declare_parameter(
            'output_topic', '/ackermann_cmd'
        ).value
        self.frame_id = self.declare_parameter(
            'frame_id', 'odom_combined'
        ).value

        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, output_topic, qos
        )
        self.subscription = self.create_subscription(
            Twist, input_topic, self.cmd_callback, qos
        )

    def cmd_callback(self, data):
        speed, steering = ackermann_command(
            data.linear.x,
            data.angular.z,
            self.wheelbase,
            self.max_steering_angle,
        )

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive.steering_angle = steering
        msg.drive.speed = speed
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVel2AckermannDriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
