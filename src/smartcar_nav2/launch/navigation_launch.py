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

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
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
    lifecycle_manager_delay_sec,
    log_level,
):
    delay_text = lifecycle_manager_delay_sec.perform(context).strip()
    try:
        delay_sec = float(delay_text)
    except ValueError as error:
        raise RuntimeError(
            "lifecycle_manager_delay_sec must be a non-negative finite number"
        ) from error
    if not math.isfinite(delay_sec) or delay_sec < 0.0:
        raise RuntimeError(
            "lifecycle_manager_delay_sec must be a non-negative finite number"
        )

    lifecycle_nodes = list(CORE_LIFECYCLE_NODES)

    parameters = [{
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'node_names': lifecycle_nodes,
    }]
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=parameters,
    )
    if delay_sec == 0.0:
        return [lifecycle_manager]
    return [TimerAction(period=delay_sec, actions=[lifecycle_manager])]


def _navigation_node_actions(
    context,
    *,
    use_sim_time,
    nav2_params_file,
    nav2_params_overlay_file,
    allow_params_overlay,
    use_respawn,
    log_level,
):
    """Create Nav2 nodes from a resolved base file and optional overlay.

    Nav2 1.1.20 on TROS cannot reliably consume the old RewrittenYaml chain.
    The base file is therefore generated at build time, while a launch caller
    may add a small, fully-resolved overlay for simulation-only behavior.
    """
    # This file is generated at build time with all BT paths resolved.  On
    # TROS Nav2 1.1.20, wrapping it in ParameterFile can lose nested
    # controller parameters and make controller_server fall back to DWB.
    parameters = [nav2_params_file.perform(context)]
    overlay_path = nav2_params_overlay_file.perform(context).strip()
    if overlay_path:
        if not _as_bool(
            context, allow_params_overlay, "allow_params_overlay"
        ):
            raise RuntimeError(
                "nav2_params_overlay_file requires allow_params_overlay=true")
        parameters.append(overlay_path)
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    lifecycle_manager_delay_sec = LaunchConfiguration(
        'lifecycle_manager_delay_sec')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_params_overlay_file = LaunchConfiguration('nav2_params_overlay_file')
    allow_params_overlay = LaunchConfiguration('allow_params_overlay')
    use_keepout_filter = LaunchConfiguration('use_keepout_filter')
    keepout_mask_yaml = LaunchConfiguration('keepout_mask_yaml')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params_fixed.yaml'),
        description='Resolved Nav2 parameter file for all launched nodes',
    )
    declare_nav2_params_overlay_file_cmd = DeclareLaunchArgument(
        'nav2_params_overlay_file',
        default_value='',
        description=(
            'Optional resolved parameter overlay applied after nav2_params_file'
        ),
    )
    declare_allow_params_overlay_cmd = DeclareLaunchArgument(
        'allow_params_overlay',
        default_value='false',
        description=(
            'Allow a caller-provided Nav2 parameter overlay. Only controlled '
            'simulation and the fixed depth obstacle overlay should enable it.'
        ),
    )
    declare_use_keepout_filter_cmd = DeclareLaunchArgument(
        'use_keepout_filter',
        default_value='true',
        description=(
            'Start the field keepout-mask stack and wait for it before Nav2. '
            'Only the self-contained Gazebo launch may disable this.'
        ),
    )
    declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
        'keepout_mask_yaml',
        default_value=os.path.join(pkg_dir, 'maps', 'field_map.yaml'),
        description='Keepout mask metadata in the odom_combined field frame',
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )
    declare_lifecycle_manager_delay_cmd = DeclareLaunchArgument(
        'lifecycle_manager_delay_sec',
        default_value='0.0',
        description=(
            'Wall-clock delay before starting the Nav2 lifecycle manager; '
            'use 0.0 for the normal bringup order'
        ),
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
            'nav2_params_file': nav2_params_file,
            'nav2_params_overlay_file': nav2_params_overlay_file,
            'allow_params_overlay': allow_params_overlay,
            'use_respawn': use_respawn,
            'log_level': log_level,
        },
    )

    lifecycle_manager = OpaqueFunction(
        function=_lifecycle_manager_actions,
        kwargs={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'lifecycle_manager_delay_sec': lifecycle_manager_delay_sec,
            'log_level': log_level,
        },
    )

    nav2_released = False

    def release_nav2_once(context):
        """Start Nav2 once, after the filter-info server is active."""
        nonlocal nav2_released
        if nav2_released:
            return []
        nav2_released = True
        return [load_nodes, lifecycle_manager]

    def keepout_gated_navigation(context):
        """Make the rule mask a Nav2 startup prerequisite on the real car."""
        if not _as_bool(context, use_keepout_filter, 'use_keepout_filter'):
            return [load_nodes, lifecycle_manager]

        mask_yaml = keepout_mask_yaml.perform(context).strip()
        if not mask_yaml or not os.path.isfile(mask_yaml):
            raise RuntimeError(
                'keepout_mask_yaml must name an existing field mask when '
                'use_keepout_filter is true'
            )
        sim_time = _as_bool(context, use_sim_time, 'use_sim_time')
        keepout_mask_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_mask_server',
            output='screen',
            parameters=[{
                'use_sim_time': sim_time,
                'yaml_filename': mask_yaml,
                'frame_id': 'odom_combined',
                'topic_name': '/keepout_filter_mask',
            }],
        )
        keepout_filter_info_server = LifecycleNode(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            namespace='',
            name='keepout_filter_info_server',
            output='screen',
            parameters=[{
                'use_sim_time': sim_time,
                'type': 0,
                'filter_info_topic': '/keepout_filter_info',
                'mask_topic': '/keepout_filter_mask',
                'base': 0.0,
                'multiplier': 1.0,
            }],
        )
        keepout_lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_keepout',
            output='screen',
            parameters=[{
                'use_sim_time': sim_time,
                'autostart': True,
                'bond_timeout': 20.0,
                'node_names': [
                    'keepout_mask_server',
                    'keepout_filter_info_server',
                ],
            }],
        )
        start_keepout_manager = RegisterEventHandler(
            OnProcessStart(
                target_action=keepout_filter_info_server,
                on_start=[TimerAction(
                    period=2.0,
                    actions=[keepout_lifecycle_manager],
                )],
            )
        )
        release_after_keepout = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=keepout_filter_info_server,
                goal_state='active',
                entities=[OpaqueFunction(function=release_nav2_once)],
            )
        )
        return [
            start_keepout_manager,
            release_after_keepout,
            keepout_mask_server,
            keepout_filter_info_server,
        ]

    load_gated_navigation = OpaqueFunction(function=keepout_gated_navigation)

    launch_description = LaunchDescription()
    launch_description.add_action(stdout_linebuf_envvar)
    launch_description.add_action(declare_use_sim_time_cmd)
    launch_description.add_action(declare_nav2_params_file_cmd)
    launch_description.add_action(declare_nav2_params_overlay_file_cmd)
    launch_description.add_action(declare_allow_params_overlay_cmd)
    launch_description.add_action(declare_use_keepout_filter_cmd)
    launch_description.add_action(declare_keepout_mask_yaml_cmd)
    launch_description.add_action(declare_autostart_cmd)
    launch_description.add_action(declare_lifecycle_manager_delay_cmd)
    launch_description.add_action(declare_use_respawn_cmd)
    launch_description.add_action(declare_log_level_cmd)
    launch_description.add_action(load_gated_navigation)
    return launch_description
