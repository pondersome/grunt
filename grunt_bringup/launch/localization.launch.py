import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_localization_nodes(context, *args, **kwargs):
    """
    Creates localization nodes with dynamically resolved prefixed frame IDs.
    Based on the canonical dual_ekf_navsat_example from robot_localization.
    """
    prefix = LaunchConfiguration('prefix').perform(context)

    # Construct prefixed frame IDs
    map_frame = f"{prefix}/map"
    odom_frame = f"{prefix}/odom"
    base_link_frame = f"{prefix}/base_link"

    config_dir = os.path.join(
        get_package_share_directory('grunt_bringup'), 'config'
    )

    ekf_local_config = os.path.join(config_dir, 'ekf_local.yaml')
    ekf_global_config = os.path.join(config_dir, 'ekf_global.yaml')
    navsat_config = os.path.join(config_dir, 'navsat_transform.yaml')

    # Local EKF: wheel odom velocities + IMU -> odom -> base_link
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        parameters=[
            ekf_local_config,
            {
                'odom_frame': odom_frame,
                'base_link_frame': base_link_frame,
                'world_frame': odom_frame,
            }
        ],
        remappings=[
            ('odom_raw', 'pose'),
            ('imu/data', f'/{prefix}/bno055/imu'),
            ('odometry/filtered', 'odometry/local'),
            ('set_pose', 'set_pose_local'),
        ],
    )

    # navsat_transform_node: GPS + IMU + local odom -> GPS odometry
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[navsat_config],
        remappings=[
            ('imu', f'/{prefix}/bno055/imu'),
            # Subscribe to the supervisor's gated fix topic, not raw rtk/fix.
            # The supervisor only republishes fixes that pass quality checks,
            # preventing UTM zone errors on garbage indoor fixes before datum.
            ('gps/fix', f'/{prefix}/rtk/fix_gated'),
            ('odometry/filtered', 'odometry/local'),
            ('odometry/gps', 'odometry/gps'),
            ('gps/filtered', 'gps/filtered'),
        ],
    )

    # Global EKF: wheel odom velocities + IMU + GPS -> map -> odom
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        parameters=[
            ekf_global_config,
            {
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_link_frame': base_link_frame,
                'world_frame': map_frame,
            }
        ],
        remappings=[
            ('odom_raw', 'pose'),
            ('imu/data', f'/{prefix}/bno055/imu'),
            ('odometry/filtered', 'odometry/global'),
        ],
    )

    # Localization supervisor: monitors sensor readiness, publishes status
    supervisor_node = Node(
        package='grunt_bringup',
        executable='localization_supervisor',
        name='localization_supervisor',
        parameters=[{
            # Quality ordering: none=0, gps=1, sbas=2, dgps=3, rtk_float=4, rtk_fixed=5
            'gps_quality_threshold': int(LaunchConfiguration('gps_quality_threshold').perform(context)),
            'gps_h_acc_cal_threshold_m': float(LaunchConfiguration('gps_h_acc_cal_threshold_m').perform(context)),
            'heading_calibration_distance': 3.0,
            'mag_calibration_threshold': 2,
            'auto_datum': True,
            'allow_re_datum': True,
            're_datum_cooldown': 30.0,
            'map_frame': map_frame,
        }],
        remappings=[
            ('rtk/fix', f'/{prefix}/rtk/fix'),
            ('rtk/navpvt', f'/{prefix}/rtk/navpvt'),
            ('bno055/imu', f'/{prefix}/bno055/imu'),
            ('bno055/calib_status', f'/{prefix}/bno055/calib_status'),
        ],
    )

    return [ekf_local_node, navsat_node, ekf_global_node, supervisor_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='mybot1',
            description='Namespace prefix for frame IDs (e.g., grunt1)'
        ),
        DeclareLaunchArgument(
            'gps_quality_threshold',
            default_value='4',
            description='Min GPS quality for heading cal: none=0, gps=1, sbas=2, dgps=3, rtk_float=4, rtk_fixed=5'
        ),
        DeclareLaunchArgument(
            'gps_h_acc_cal_threshold_m',
            default_value='0.5',
            description='Max horizontal accuracy (m) for heading calibration start/end'
        ),
        OpaqueFunction(function=create_localization_nodes),
    ])
