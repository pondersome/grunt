"""
Nav2 INDOOR-mode launch for grunt — tight-space navigation, MPPI
controller with obstacle-aware trajectory sampling, smaller costmaps,
odom frame (no GPS/map).

One of two per-mode launch files in grunt_nav. Dispatched by
nav.launch.py based on the `nav_mode` argument. The node topology
is identical to nav_outdoor.launch.py — only the config YAML
differs (nav2_indoor.yaml vs nav2_outdoor.yaml), and the indoor
config in turn points at a different BT XML.

A future refactor could share the Nav2-node-setup fragment between
indoor and outdoor launches (they differ only in the default
params file). Not worth it at Phase B scale — keeping them separate
lets them diverge cleanly when indoor-specific launch wiring is
needed (sensor subscriptions, mode-specific lifecycle configs, etc).
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
    grunt_nav_dir = get_package_share_directory('grunt_nav')

    prefix = LaunchConfiguration('prefix')
    params_file = LaunchConfiguration('nav2_params_file')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [('/tf', '/tf'), ('/tf_static', '/tf_static')]
    # Same cmd_vel chain as outdoor — see nav_outdoor.launch.py for the
    # full rationale. Summary: controller + behavior_server both publish
    # to cmd_vel_nav_raw; collision_monitor gates to cmd_vel_nav;
    # twist_mux consumes cmd_vel_nav at priority 50.
    controller_cmd_vel_remappings = remappings + [
        ('cmd_vel', ['/', prefix, '/cmd_vel_nav_raw']),
    ]
    behavior_cmd_vel_remappings = controller_cmd_vel_remappings
    cmd_vel_remappings = controller_cmd_vel_remappings
    base_remappings = remappings

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
                        remappings=controller_cmd_vel_remappings + [
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
                    Node(
                        package='nav2_collision_monitor',
                        executable='collision_monitor',
                        name='collision_monitor',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[configured_params],
                        remappings=remappings + [
                            ('cmd_vel_in', ['/', prefix, '/cmd_vel_nav_raw']),
                            ('cmd_vel_out', ['/', prefix, '/cmd_vel_nav']),
                        ],
                    ),
                    # Periodic local_costmap clear-around — works around
                    # sparse-sensor stale-mark accumulation (see
                    # memory/project_costmap_hygiene_backlog.md).
                    # Sits inside the 'nav' PushRosNamespace so its
                    # relative service_name resolves to
                    # /<prefix>/nav/local_costmap/clear_around_local_costmap.
                    # Not a lifecycle node — runs independently of the
                    # Nav2 lifecycle_manager, tolerates the service
                    # being transiently unavailable.
                    Node(
                        package='grunt_nav',
                        executable='clear_costmap_timer',
                        name='clear_costmap_timer',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[{
                            'rate_hz': 5.0,
                            'reset_distance': 4.0,
                        }],
                    ),
                ]
            ),
        ]
    )

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
            default_value=os.path.join(grunt_nav_dir, 'config', 'nav2_indoor.yaml'),
            description='Nav2 indoor parameters file'
        ),
        nav2_nodes,
        nav2_lifecycle,
    ])
