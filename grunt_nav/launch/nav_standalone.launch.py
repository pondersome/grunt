"""
Standalone Nav2 launch for grunt — entry point for the grunt-nav@.service
systemd unit.

Wraps nav.launch.py (the per-mode dispatcher) in a PushRosNamespace so
Nav2 topics land at /<prefix>/nav/... even when launched independently
of grunt.quiet.launch.py.

Why this exists separately from nav.launch.py:
    grunt.quiet.launch.py wraps everything (p2os, EKFs, Nav2, etc) in a
    PushRosNamespace(prefix) parent group. When Nav2 was included by
    grunt.quiet.launch.py it inherited that namespace for free.
    Decoupling Nav2 into its own systemd unit means the outer prefix
    namespace has to be applied here instead — otherwise Nav2 would
    land at /nav/... (unnamespaced) and every topic/service would drift
    off the rest of the stack.

Dev workflow:
    Terminal 1: ros2 launch grunt_bringup grunt.quiet.launch.py
    Terminal 2: ros2 launch grunt_nav nav_standalone.launch.py \\
                  prefix:=grunt1 nav_mode:=indoor

Production workflow:
    grunt.service starts the main stack (no Nav2).
    grunt-nav@indoor.service (or outdoor) starts Nav2 in the chosen mode.
    nav_mode_switcher (in the main stack) stops/starts these units to
    switch modes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    prefix = LaunchConfiguration('prefix')
    nav_mode = LaunchConfiguration('nav_mode')

    grunt_nav_share = FindPackageShare('grunt_nav')

    nav_group = GroupAction([
        PushRosNamespace(prefix),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([grunt_nav_share, 'launch', 'nav.launch.py'])
            ),
            launch_arguments={
                'prefix': prefix,
                'nav_mode': nav_mode,
            }.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix', default_value='grunt1',
            description='Robot namespace prefix'),
        DeclareLaunchArgument(
            'nav_mode', default_value='outdoor',
            description='Nav2 configuration mode: outdoor (RPP, GPS) or indoor (MPPI)'),
        nav_group,
    ])
