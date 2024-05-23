from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
  
        # Node for joystick control
        Node(
            package='joy',
            executable='joy_node',
            name='logitech_joy_controller',
            #parameters=[joystick_config]
            parameters = [{'dev': '/dev/input/js1'}]
        ),
    ])
