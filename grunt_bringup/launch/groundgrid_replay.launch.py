# GroundGrid replay / live launch — phase A of task #95.
#
# Pairs with our local patch to upstream GroundGridNode that parameterizes
# the three hardcoded TF frame names (base_link, odom, velodyne). Passes
# grunt's namespaced frames directly — no static-transform aliasing.
#
# Live mode:
#   ros2 launch grunt_bringup groundgrid_replay.launch.py use_sim_time:=false
#
# Bag mode (stop grunt.service first, then in two terminals):
#   ros2 launch grunt_bringup groundgrid_replay.launch.py
#   ros2 bag play --clock /home/karim/bag_files/grunt/2026-05-28_11-31-45_run_gps_mission

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # L2-tuned config — phase B starting point. See header comments
    # in the yaml for the rationale on each non-default pick.
    config_file = os.path.join(
        get_package_share_directory('grunt_bringup'),
        'config', 'groundgrid_l2.yaml',
    )

    pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/grunt1/l2/points_obstacles',
        description='Input PointCloud2 topic',
    )
    odometry_topic = DeclareLaunchArgument(
        'odometry_topic',
        default_value='/grunt1/odometry/local',
        description='Input Odometry topic (EKF local)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='True for bag replay (set --clock on the bag); false for live grunt',
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
            # Local frame-name patch — see GroundGridNode.cpp constructor.
            {'groundgrid/base_frame_id': 'grunt1/base_link'},
            {'groundgrid/odom_frame_id': 'grunt1/odom'},
            {'groundgrid/lidar_frame_id': 'grunt1/mast/l2'},
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
        groundgrid_node,
    ])
