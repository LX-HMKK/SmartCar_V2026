import unittest

import launch
import launch_testing
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


class TestSmartcarNav2Launch(unittest.TestCase):

    def test_count_until_timeout(self, proc_output):
        # launch 启动后 5 秒内应能完成初始化并稳定运行
        proc_output.assertWaitFor(
            'Nav2 lifecycle manager completed', timeout=5, stream='stdout')


def generate_test_description():
    pkg_dir = get_package_share_directory('smartcar_nav2')
    launch_file = os.path.join(pkg_dir, 'launch', 'smartcar_nav2.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'use_sim_time': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])
