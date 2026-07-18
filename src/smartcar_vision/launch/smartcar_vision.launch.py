"""Launch the selected camera, zbar reader, and SmartCar vision services."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DRIVER_TOPICS = {
    "aurora": "/aurora/rgb/image_raw",
    "usb": "/image",
    "mipi": "/image_raw",
}
VALID_DRIVERS = ("aurora", "usb", "mipi", "none")


def _camera_action(camera_driver, usb_video_device):
    if camera_driver == "aurora":
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("deptrum-ros-driver-aurora930"),
                "launch",
                "aurora930_launch.py",
            ])),
            launch_arguments={
                "rgb_enable": "true",
                "rgb_fps": "15",
                "ir_fps": "15",
                "depth_enable": "false",
                "ir_enable": "false",
                "point_cloud_enable": "false",
                "rgbd_enable": "false",
                "align_mode": "false",
                "depth_correction": "false",
            }.items(),
        )
    if camera_driver == "usb":
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("hobot_usb_cam"),
                "launch",
                "hobot_usb_cam.launch.py",
            ])),
            launch_arguments={
                "usb_video_device": usb_video_device,
                "usb_framerate": "15",
                "usb_image_width": "640",
                "usb_image_height": "480",
                "usb_pixel_format": "mjpeg2rgb",
                "usb_zero_copy": "False",
            }.items(),
        )
    if camera_driver == "mipi":
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("mipi_cam"),
                "launch",
                "mipi_cam_640x480_bgr8.launch.py",
            ])),
            launch_arguments={
                "mipi_io_method": "ros",
                "mipi_out_format": "bgr8",
                "mipi_image_width": "640",
                "mipi_image_height": "480",
            }.items(),
        )
    return None


def _runtime_actions(context):
    camera_driver = LaunchConfiguration("camera_driver").perform(context).strip()
    configured_topic = LaunchConfiguration("image_topic").perform(context).strip()
    if camera_driver not in VALID_DRIVERS:
        raise RuntimeError(
            "camera_driver must be one of aurora, usb, mipi, or none")
    if camera_driver == "none":
        if not configured_topic:
            raise RuntimeError(
                "image_topic must be provided when camera_driver is none")
        source_topic = configured_topic
    else:
        source_topic = configured_topic or DRIVER_TOPICS[camera_driver]

    actions = []
    camera_action = _camera_action(
        camera_driver,
        LaunchConfiguration("usb_video_device").perform(context),
    )
    if camera_action is not None:
        actions.append(camera_action)

    actions.extend([
        Node(
            package="smartcar_vision",
            executable="vision_node",
            name="vision_node",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "image_topic": source_topic,
                },
            ],
        ),
        Node(
            package="zbar_ros",
            executable="barcode_reader",
            name="barcode_reader",
            output="screen",
            parameters=[{"throttle_repeated_barcodes": 0.0}],
            remappings=[
                ("image", source_topic),
                ("barcode", "/barcode"),
            ],
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_driver",
            default_value="aurora",
            description="Camera driver: aurora, usb, mipi, or none",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="",
            description="Override the selected driver's image topic",
        ),
        DeclareLaunchArgument(
            "usb_video_device",
            default_value="/dev/video0",
            description="USB camera device used by the optional fallback",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_vision"),
                "config",
                "vision.yaml",
            ]),
            description="Vision service parameter file",
        ),
        OpaqueFunction(function=_runtime_actions),
    ])
