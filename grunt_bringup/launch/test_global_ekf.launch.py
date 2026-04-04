"""
Minimal launch for testing global EKF against bag playback.
Run with: ros2 launch grunt_bringup test_global_ekf.launch.py

Requires bag playback in another terminal:
  ros2 bag play datum_debug2 --clock --topics /grunt1/pose /grunt1/bno055/imu \
    /grunt1/rtk/fix /grunt1/rtk/navpvt /grunt1/odometry/local /tf /tf_static
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    prefix = 'grunt1'
    map_frame = f'{prefix}/map'
    odom_frame = f'{prefix}/odom'
    base_link_frame = f'{prefix}/base_link'

    config_dir = os.path.join(
        get_package_share_directory('grunt_bringup'), 'config'
    )

    return LaunchDescription([
        # Static TF from URDF: base_link -> chassis -> top_plate -> mast -> gps_link
        # Bag replay may miss /tf_static due to QoS durability.
        # This provides just the direct base_link->gps_link path needed by navsat_transform.
        # Offset from the actual URDF: mast at -0.10 on top_plate, gps at 0.42m up on mast
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_static_tf',
            arguments=[
                '--x', '-0.097', '--y', '0', '--z', '0.694',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', f'{prefix}/base_link',
                '--child-frame-id', f'{prefix}/mast/gps_link',
            ],
            parameters=[{'use_sim_time': True}],
        ),

        # Also provide the imu_link static TF in case navsat needs it
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            arguments=[
                '--x', '0', '--y', '-0.05', '--z', '0.494',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', f'{prefix}/base_link',
                '--child-frame-id', f'{prefix}/mast/imu_link',
            ],
            parameters=[{'use_sim_time': True}],
        ),

        # navsat_transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            namespace=prefix,
            parameters=[
                os.path.join(config_dir, 'navsat_transform.yaml'),
                {'use_sim_time': True},
            ],
            remappings=[
                ('imu', f'/{prefix}/bno055/imu'),
                ('gps/fix', f'/{prefix}/rtk/fix'),
                ('odometry/filtered', f'/{prefix}/odometry/local'),
                ('odometry/gps', f'/{prefix}/odometry/gps_test'),
                ('gps/filtered', f'/{prefix}/gps/filtered_test'),
            ],
        ),

        # Global EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            namespace=prefix,
            parameters=[
                os.path.join(config_dir, 'ekf_global.yaml'),
                {
                    'map_frame': map_frame,
                    'odom_frame': odom_frame,
                    'base_link_frame': base_link_frame,
                    'world_frame': map_frame,
                    'use_sim_time': True,
                }
            ],
            remappings=[
                ('odom_raw', f'/{prefix}/pose'),
                ('imu/data', f'/{prefix}/bno055/imu'),
                ('odometry/gps', f'/{prefix}/odometry/gps_test'),
                ('odometry/filtered', f'/{prefix}/odometry/global_test'),
            ],
        ),
    ])
