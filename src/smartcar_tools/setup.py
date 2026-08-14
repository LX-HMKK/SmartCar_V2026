from glob import glob
import os

from setuptools import find_packages, setup


package_name = "smartcar_tools"


def install_tree(source_directory):
    """Return data_files entries while preserving nested config paths."""
    entries = []
    for source in glob(source_directory + "/**/*", recursive=True):
        if os.path.isfile(source):
            relative_parent = os.path.dirname(source)
            entries.append((
                os.path.join("share", package_name, relative_parent),
                [source],
            ))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ] + install_tree("config") + install_tree("rviz"),
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="LX-HMKK",
    maintainer_email="lx_hmkk@qq.com",
    description="Field reference, waypoint editing, diagnostics, vision, and speech tools.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "speech_probe = smartcar_tools.speech_probe:main",
            "qr_probe = smartcar_tools.qr_probe:main",
            "image_replay_node = smartcar_tools.image_replay_node:main",
            "rgb_imshow = smartcar_tools.rgb_imshow:main",
            "competition_output_display = "
            "smartcar_tools.competition_output_display:main",
            "vlm_display = smartcar_tools.vlm_display:main",
            "waypoint_viz = smartcar_tools.waypoint_viz:main",
            "field_reference_node = smartcar_tools.field_reference_node:main",
            "odom_diag = smartcar_tools.odom_diag:main",
            "waypoint_drag_editor = smartcar_tools.waypoint_drag_editor:main",
            "voltage_monitor = smartcar_tools.voltage_monitor:main",
            "steering_circle_analyze = smartcar_tools.steering_circle_analyze:main",
            "steering_circle_drive = smartcar_tools.steering_circle_drive:main",
            "steering_hold = smartcar_tools.steering_hold:main",
            "short_drive_test = smartcar_tools.short_drive_test:main",
        ],
    },
)
