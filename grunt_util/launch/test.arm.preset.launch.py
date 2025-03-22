#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument to pass in a prefix value
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='/mybot/arm1/',
        description='Prefix for the joint names'
    )

    # Launch the arm preset node with the provided prefix
    arm_node = Node(
        package='grunt_util',  # Replace with your actual package name
        executable='arm_preset_publisher.py',  # Ensure this matches your node executable
        name='arm_preset_publisher',
        parameters=[{'prefix': LaunchConfiguration('prefix')}],
        output='screen',
        remappings=[('joint_states', 'mybot/joint_states')]
    )

    # Use a TimerAction to delay publishing the initial preset command to ensure the node is running
    publish_initial_preset = TimerAction(
        period=2.0,  # Delay in seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/arm_preset',
                    'std_msgs/msg/String',
                    "{data: 'nav_forward'}"
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        prefix_arg,
        arm_node,
        publish_initial_preset,
    ])
