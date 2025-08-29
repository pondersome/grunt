#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Declare a launch argument to pass in a prefix value
    # This values gets used to both set the prefix for the joint names in the TF tree, as
    # well as to set the common namespace for the node and its pubs and subs
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/grunt1',
        description='Namespace for robot'
    )

    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='/arm1/',
        description='Prefix for joint names and sub-namespace'
    )

    # Optional: if provided, publish this preset once after startup
    initial_preset_arg = DeclareLaunchArgument(
        'initial_arm_preset',
        default_value='',
        description='If non-empty, publish this arm preset once on startup (e.g., "lookout@forward")'
    )

    # Launch the arm preset node with the provided namespace and prefix - these combine into a single prefix for the node
    arm_node = Node(
        package='grunt_util',  # Replace with your actual package name
        executable='arm_preset_publisher.py',  # Ensure this matches your node executable
        name='arm_preset_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{'prefix': [LaunchConfiguration('namespace'), LaunchConfiguration('prefix')]}],
        output='screen',
        #remappings=[('joint_states', 'mybot/joint_states')]
    )

    # Optionally publish an initial preset once after startup if provided
    publish_initial_preset = TimerAction(
        period=2.0,  # Delay in seconds to ensure subscriber is ready
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    [LaunchConfiguration('namespace'), '/arm_preset'],
                    'std_msgs/msg/String',
                    ["{data: '", LaunchConfiguration('initial_arm_preset'), "'}"]
                ],
                output='screen'
            )
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('initial_arm_preset'), "' != ''"]))
    )

    return LaunchDescription([
        namespace_arg,
        prefix_arg,
        initial_preset_arg,
        arm_node,
        publish_initial_preset,
    ])
