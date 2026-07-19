from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            parameters=[{
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_laser",
                "publish_tf": False,
                "base_frame_id": "base_footprint",
                "odom_frame_id": "odom_combined",
                "init_pose_from_topic": "",
                "reset_service": "/smartcar/localization/reset_laser_odometry",
                "freq": 10.0,
            }],
        ),
    ])
