# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Nav2 Humble 1.1.20 navigation launch with a single smoothed command path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


CORE_LIFECYCLE_NODES = (
    'controller_server',
    'planner_server',
    'bt_navigator',
    'velocity_smoother',
)


def _as_bool(context, substitution, name):
    value = substitution.perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(f'{name} must be true or false')


def _lifecycle_manager_actions(
    context,
    *,
    use_sim_time,
    autostart,
    log_level,
):
    lifecycle_nodes = list(CORE_LIFECYCLE_NODES)

    parameters = [{
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'node_names': lifecycle_nodes,
    }]
    return [Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=parameters,
    )]


def _navigation_node_actions(
    context,
    *,
    use_sim_time,
    params_file,
    params_overlay_file,
    use_respawn,
    log_level,
):
    """Create Nav2 nodes from a resolved base file and optional overlay.

    Nav2 1.1.20 on TROS cannot reliably consume the old RewrittenYaml chain.
    The base file is therefore generated at build time, while a launch caller
    may add a small, fully-resolved overlay for simulation-only behavior.
    """
    parameters = [ParameterFile(
        params_file.perform(context),
        allow_substs=True,
    )]
    overlay_path = params_overlay_file.perform(context).strip()
    if overlay_path:
        parameters.append(ParameterFile(overlay_path, allow_substs=True))
    parameters.append({
        "use_sim_time": _as_bool(context, use_sim_time, "use_sim_time"),
    })

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    return [GroupAction(
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=parameters,
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=parameters,
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=parameters,
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=parameters,
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel_candidate'),
                ],
            ),
        ],
    )]


def generate_launch_description():
    pkg_dir = get_package_share_directory('smartcar_nav2')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    params_overlay_file = LaunchConfiguration('params_overlay_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params_fixed.yaml'),
        description='Resolved Nav2 parameter file for all launched nodes',
    )
    declare_params_overlay_file_cmd = DeclareLaunchArgument(
        'params_overlay_file',
        default_value='',
        description='Optional resolved parameter overlay applied after params_file',
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Respawn crashed nodes when composition is disabled',
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level'
    )

    load_nodes = OpaqueFunction(
        function=_navigation_node_actions,
        kwargs={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'params_overlay_file': params_overlay_file,
            'use_respawn': use_respawn,
            'log_level': log_level,
        },
    )

    lifecycle_manager = OpaqueFunction(
        function=_lifecycle_manager_actions,
        kwargs={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'log_level': log_level,
        },
    )

    launch_description = LaunchDescription()
    launch_description.add_action(stdout_linebuf_envvar)
    launch_description.add_action(declare_namespace_cmd)
    launch_description.add_action(declare_use_sim_time_cmd)
    launch_description.add_action(declare_params_file_cmd)
    launch_description.add_action(declare_params_overlay_file_cmd)
    launch_description.add_action(declare_autostart_cmd)
    launch_description.add_action(declare_use_respawn_cmd)
    launch_description.add_action(declare_log_level_cmd)
    launch_description.add_action(load_nodes)
    launch_description.add_action(lifecycle_manager)
    return launch_description
