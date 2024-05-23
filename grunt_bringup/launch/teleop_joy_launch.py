from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare the parameters for teleop_base node
        Node(
            package='p2os_teleop',
            executable='p2os_teleop',
            name='p2os_teleop',
            remappings=[
                ('/des_vel', '/base_controller/command')
            ],
            parameters=[
                {'axis_vx': 1},
                {'axis_vw': 2},
                {'axis_vy': 0},
                {'deadman_button': 5},
                {'run_button': 4}
            ]
        ),

        # Node for joystick control with device parameter from YAML
        Node(
            package='joy',
            executable='joy_node',
            name='logitech_joy_controller',
            #parameters=[joystick_config]
            parameters = [{'dev': '/dev/input/js1'}]
        ),
    ])
