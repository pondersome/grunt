"""
Dispatcher launch file for grunt_nav.

Routes to nav_indoor.launch.py or nav_outdoor.launch.py based on the
`nav_mode` launch argument. Called from nav_standalone.launch.py,
which wraps this in the appropriate PushRosNamespace(prefix).

Phase C1: mode switching is implemented at the systemd layer —
nav_mode_switcher stops/starts grunt-nav@<mode>.service units,
each of which launches nav_standalone.launch.py with a different
nav_mode argument. This dispatcher remains the single point where
the mode selection becomes a per-mode launch include.
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

    outdoor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([grunt_nav_share, 'launch', 'nav_outdoor.launch.py'])
        ),
        launch_arguments={'prefix': prefix}.items(),
        condition=IfCondition(
            PythonExpression(["'", nav_mode, "' == 'outdoor'"])
        ),
    )

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
