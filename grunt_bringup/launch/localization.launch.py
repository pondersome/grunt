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
            # fromLL stays at its natural namespaced path (/<prefix>/fromLL).
            # Nav2's waypoint_follower remaps its hardcoded absolute /fromLL
            # to this namespaced path — see nav2.launch.py.
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
            # Use the supervisor's map-frame relay instead of navsat's odom-frame
            # output. This prevents a self-referential TF feedback loop that
            # causes linear divergence on Jazzy (see followup article section 9).
            ('odometry/gps', 'odometry/gps_map'),
        ],
    )

    # navpvt_to_imu: republishes u-blox course-over-ground heading as a
    # yaw-only IMU observation for the global EKF to fuse. Eliminates the
    # one-shot drive-cal heading bias that compounds into wandering when
    # cal happens on tilted ground.
    #
    # max_head_acc_deg defaults to 15° (raised from initial 3° on
    # 2026-04-28). Field measurement on the Ranchero driveway under tree
    # cover showed median NavPVT head_acc ~27° on FLOAT-only RTK; a 3°
    # gate dropped 100% of messages for an entire 9.8-min mission segment
    # (zero corrections, heading drifted to median −72° vs ground truth).
    # 15° catches roughly half of FLOAT-quality headings while still
    # rejecting the obvious 60°+ garbage; the EKF's variance weighting
    # keeps noisy observations from over-correcting.
    navpvt_heading_node = Node(
        package='grunt_bringup',
        executable='navpvt_to_imu',
        name='navpvt_to_imu',
        parameters=[{
            'frame_id': base_link_frame,
            'min_speed_mps': 0.2,
            'max_head_acc_deg': 15.0,
            'min_carrier': 1,  # 1=FLOAT, 2=FIXED
        }],
        remappings=[
            ('rtk/navpvt', f'/{prefix}/rtk/navpvt'),
            ('rtk/heading_imu', f'/{prefix}/rtk/heading_imu'),
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
            # Optional per-site config file. If present, the supervisor reads
            # anchor_lat/anchor_lon and uses those for the /datum call instead
            # of wherever the robot happens to be at calibration time. Stable
            # cross-run map origin. Empty string disables.
            'site_config': LaunchConfiguration('site_config').perform(context),
        }],
        remappings=[
            ('rtk/fix', f'/{prefix}/rtk/fix'),
            ('rtk/navpvt', f'/{prefix}/rtk/navpvt'),
            ('bno055/imu', f'/{prefix}/bno055/imu'),
            ('bno055/calib_status', f'/{prefix}/bno055/calib_status'),
        ],
    )

    return [ekf_local_node, navsat_node, ekf_global_node,
            navpvt_heading_node, supervisor_node]


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
        DeclareLaunchArgument(
            'site_config',
            default_value='',
            description='Path to per-site site.yaml (anchor_lat/anchor_lon). Empty = use first GPS fix as datum (legacy).'
        ),
        OpaqueFunction(function=create_localization_nodes),
    ])
