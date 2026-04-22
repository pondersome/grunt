"""
Dispatcher launch file for grunt_nav.

Routes to nav_indoor.launch.py or nav_outdoor.launch.py based on the
`nav_mode` launch argument. In Phase A (current) only outdoor exists,
so this is a thin passthrough to nav_outdoor.launch.py — the stub
establishes the interface that grunt_bringup uses now and that the
future mode-switcher + nav_indoor.launch.py will hang off of.

In Phase C this file (or a sibling) will bring up BOTH nav stacks
under separate namespaces using lifecycle-based quiescence, with a
mode-switcher node flipping activation in response to environment
state. See ~/.claude/plans/sharded-mapping-sonnet.md for the
architecture plan.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    prefix = LaunchConfiguration('prefix')
    nav_mode = LaunchConfiguration('nav_mode')

    grunt_nav_share = FindPackageShare('grunt_nav')

    # For now only nav_outdoor.launch.py exists; nav_indoor arrives in
    # Phase B of the grunt_nav rollout. The dispatcher supports the
    # same `nav_mode` selector that the Phase C switcher will toggle.
    outdoor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([grunt_nav_share, 'launch', 'nav_outdoor.launch.py'])
        ),
        launch_arguments={'prefix': prefix}.items(),
        condition=IfCondition(
            PythonExpression(["'", nav_mode, "' == 'outdoor'"])
        ),
    )

    # Placeholder for Phase B. Keeps the dispatcher's interface
    # stable — callers can ask for nav_mode:=indoor today and get a
    # sensible error when the file isn't there yet, rather than having
    # us restructure callers when indoor lands.
    indoor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([grunt_nav_share, 'launch', 'nav_indoor.launch.py'])
        ),
        launch_arguments={'prefix': prefix}.items(),
        condition=IfCondition(
            PythonExpression(["'", nav_mode, "' == 'indoor'"])
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix', default_value='grunt1',
            description='Robot namespace prefix'),
        DeclareLaunchArgument(
            'nav_mode', default_value='outdoor',
            description='Nav2 configuration mode: outdoor (GPS/RPP) or indoor (MPPI) — indoor lands in Phase B'),
        outdoor,
        indoor,
    ])
