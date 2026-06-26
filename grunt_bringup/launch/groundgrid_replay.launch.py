# GroundGrid replay launch — phase A of task #95.
#
# Designed for `ros2 bag play` against the 2026-05-28 mission bags. Wires
# the upstream groundgrid_node into our namespaced TF tree by aliasing
# three frames via identity static transforms:
#   grunt1/odom      -> odom
#   grunt1/base_link -> base_link
#   grunt1/mast/l2   -> velodyne
# Upstream's GroundGridNode hardcodes those three strings (~14 places).
# Aliases unblock phase A without a source patch; phase C will parameterize.
#
# Sources the bag's already-filtered cloud (`/grunt1/l2/points_obstacles`)
# and EKF local odom (`/grunt1/odometry/local`). l2_self_filter is not
# re-run here; phase B/C will reintroduce it once we tune.
#
# Usage:
#   Terminal 1: ros2 launch grunt_bringup groundgrid_replay.launch.py
#   Terminal 2: ros2 bag play --clock /path/to/bag
#
# `--clock` is required because the launch sets use_sim_time=True so the
# node consumes the bag's recorded timestamps for TF lookups.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    groundgrid_share = get_package_share_directory('groundgrid')
    # Seed config: KITTI defaults. Phase B tunes against L2 bags.
    config_file = os.path.join(groundgrid_share, 'param', 'kitti.yaml')

    pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/grunt1/l2/points_obstacles',
        description='Input PointCloud2 topic (bag-published)',
    )
    odometry_topic = DeclareLaunchArgument(
        'odometry_topic',
        default_value='/grunt1/odometry/local',
        description='Input Odometry topic for ego pose (EKF local)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='True for bag replay (set --clock on the bag); false for live grunt',
    )

    tf_grunt_odom_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_alias_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'grunt1/odom', 'odom'],
    )
    tf_grunt_base_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_alias_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'grunt1/base_link', 'base_link'],
    )
    tf_l2_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_alias_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'grunt1/mast/l2', 'velodyne'],
    )

    groundgrid_node = Node(
        package='groundgrid',
        executable='groundgrid_node',
        name='groundgrid_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'groundgrid/dataset_name': 'live'},
            {'groundgrid/visualize': True},
            config_file,
        ],
        remappings=[
            ('/pointcloud', LaunchConfiguration('pointcloud_topic')),
            ('/groundgrid/odometry_in', LaunchConfiguration('odometry_topic')),
        ],
    )

    return LaunchDescription([
        pointcloud_topic,
        odometry_topic,
        use_sim_time_arg,
        tf_grunt_odom_to_odom,
        tf_grunt_base_to_base,
        tf_l2_to_velodyne,
        groundgrid_node,
    ])
