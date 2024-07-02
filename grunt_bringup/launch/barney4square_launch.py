from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the scripted_bot_driver node with arguments
        Node(
            package='scripted_bot_driver',
            executable='scripted_mover',
            name='scripted_mover',
            output='screen',
            arguments=[
                'drive_waypoints', '3', '0', '3', '3', '0', '3', '0', '0'
            ]
        ),

        # Start the rosbag2 recording
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', 'your_rosbag_filename'],
            output='screen'
        )
    ])