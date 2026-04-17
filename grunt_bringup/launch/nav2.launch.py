"""
Nav2 outdoor GPS waypoint following launch file for Grunt.

Customized from nav2_bringup's navigation_launch.py:
- Removed docking_server (not used)
- Removed composition mode (unnecessary on NUC)
- Added /nav sub-namespace for grouping (like /rtk for GPS nodes)
- Delayed startup to let base robot stack stabilize
- cmd_vel remapped to cmd_vel_nav (twist_mux priority 50 < joystick 100)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    grunt_bringup_dir = get_package_share_directory('grunt_bringup')

    prefix = LaunchConfiguration('prefix')
    params_file = LaunchConfiguration('nav2_params_file')

    # Nodes managed by the lifecycle manager
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        # velocity_smoother excluded — Phase 1, simplify the cmd_vel chain.
        # collision_monitor excluded — Phase 1 has no sensors, and the
        # disabled scan source causes 72% CPU spin-wait.
        'bt_navigator',
        'waypoint_follower',
    ]

    # Use absolute paths for TF and cmd_vel so the /nav sub-namespace
    # doesn't redirect them away from the main robot topics
    remappings = [('/tf', '/tf'), ('/tf_static', '/tf_static')]
    # cmd_vel remappings need absolute path to reach twist_mux at /<prefix>/cmd_vel_nav
    cmd_vel_remappings = remappings + [('cmd_vel', ['/', prefix, '/cmd_vel_nav'])]
    # Nodes that don't publish cmd_vel just get TF remappings
    base_remappings = remappings

    # Two-stage param preparation:
    #   1. ReplaceString substitutes the `grunt1` placeholder in the yaml
    #      with the actual prefix (so frame IDs and topic paths are
    #      correct for multi-robot networks).
    #   2. RewrittenYaml wraps the result to inject `autostart: true`
    #      and scope params under the full namespace (prefix/nav).
    prefixed_params = ReplaceString(
        source_file=params_file,
        replacements={'grunt1': prefix},
    )
    nav_namespace = [prefix, '/nav']
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=prefixed_params,
            root_key=nav_namespace,
            param_rewrites={'autostart': 'true'},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # All Nav2 nodes in a group with delayed startup
    # Note: period must be a float, not a LaunchConfiguration, because
    # TimerAction evaluates the substitution at include time
    nav2_nodes = TimerAction(
        period=20.0,
        actions=[
            GroupAction(
                actions=[
                    PushRosNamespace('nav'),
                    SetParameter('use_sim_time', False),
                    Node(
                        package='nav2_controller',
                        executable='controller_server',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[configured_params],
                        remappings=cmd_vel_remappings + [
                            ('odom', ['/', prefix, '/odometry/local']),
                        ],
                    ),
                    Node(
                        package='nav2_smoother',
                        executable='smoother_server',
                        name='smoother_server',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[configured_params],
                        remappings=base_remappings,
                    ),
                    Node(
                        package='nav2_planner',
                        executable='planner_server',
                        name='planner_server',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[configured_params],
                        remappings=base_remappings,
                    ),
                    Node(
                        package='nav2_route',
                        executable='route_server',
                        name='route_server',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[configured_params],
                        remappings=base_remappings,
                    ),
                    Node(
                        package='nav2_behaviors',
                        executable='behavior_server',
                        name='behavior_server',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[configured_params],
                        remappings=cmd_vel_remappings,
                    ),
                    Node(
                        package='nav2_bt_navigator',
                        executable='bt_navigator',
                        name='bt_navigator',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[configured_params],
                        remappings=base_remappings,
                    ),
                    Node(
                        package='nav2_waypoint_follower',
                        executable='waypoint_follower',
                        name='waypoint_follower',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[configured_params],
                        remappings=base_remappings + [
                            ('/fromLL', ['/', prefix, '/fromLL']),
                        ],
                    ),
                    # velocity_smoother excluded for Phase 1 — simplifies
                    # the cmd_vel chain. Controller publishes directly to
                    # twist_mux via /grunt1/cmd_vel_nav.
                    # collision_monitor excluded for Phase 1 — no sensors,
                    # and disabled scan source causes 72% CPU spin-wait.
                    # Re-add when LiDAR obstacle detection is configured.
                ]
            ),
        ]
    )

    # Lifecycle manager starts separately, 5s after the other Nav2 nodes,
    # so bt_navigator has time to finish loading BT plugins before the
    # manager tries to configure it. Without this delay, the NUC's quad-core
    # can't finish plugin loading before lifecycle_manager's service call
    # times out, and the entire Nav2 stack fails to come up.
    nav2_lifecycle = TimerAction(
        period=25.0,
        actions=[
            GroupAction(
                actions=[
                    PushRosNamespace('nav'),
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_navigation',
                        output='screen',
                        parameters=[
                            {'autostart': True},
                            {'node_names': lifecycle_nodes},
                            {'bond_timeout': 10.0},
                            {'bond_respawn_max_duration': 60.0},
                        ],
                    ),
                ]
            ),
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'prefix', default_value='grunt1',
            description='Robot namespace prefix'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(grunt_bringup_dir, 'config', 'nav2_outdoor_params.yaml'),
            description='Nav2 parameters file'
        ),
        nav2_nodes,
        nav2_lifecycle,
    ])
