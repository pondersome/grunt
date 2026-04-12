import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def create_p2os_node(context, *args, **kwargs):
    """
    Creates the p2os_driver node with dynamically resolved frame IDs
    and configurable velocity/acceleration parameters.
    When Localization is active, disables TF broadcast and keeps pose
    on its native topic so robot_localization can consume it.
    """
    prefix = LaunchConfiguration('prefix').perform(context)
    use_sonar = LaunchConfiguration('use_sonar').perform(context)
    p3at_port = LaunchConfiguration('P3AT_port').perform(context)
    localization = LaunchConfiguration('Localization').perform(context)
    use_localization = localization == '1'

    # Construct frame IDs with namespace prefix
    base_link_frame_id = f"{prefix}/base_link"
    odom_frame_id = f"{prefix}/odom"

    # Velocity and acceleration parameters
    max_xspeed = float(LaunchConfiguration('max_xspeed').perform(context))
    max_yawspeed = float(LaunchConfiguration('max_yawspeed').perform(context))
    max_xaccel = float(LaunchConfiguration('max_xaccel').perform(context))
    max_xdecel = float(LaunchConfiguration('max_xdecel').perform(context))
    max_yawaccel = float(LaunchConfiguration('max_yawaccel').perform(context))
    max_yawdecel = float(LaunchConfiguration('max_yawdecel').perform(context))
    p2os_baud_rate = int(LaunchConfiguration('p2os_baud_rate').perform(context))

    # When localization is active: suppress p2os TF, don't remap pose to odom
    # When not active: p2os broadcasts TF and pose is remapped to odom (legacy behavior)
    remappings = [] if use_localization else [('pose', 'odom')]

    return [
        Node(
            package='p2os_driver',
            executable='p2os_driver',
            name='p2os_driver',
            parameters=[{
                'use_sonar': use_sonar == 'True' or use_sonar == 'true',
                'port': p3at_port,
                'base_link_frame_id': base_link_frame_id,
                'odom_frame_id': odom_frame_id,
                'publish_tf': not use_localization,
                'max_xspeed': max_xspeed,
                'max_yawspeed': max_yawspeed,
                'max_xaccel': max_xaccel,
                'max_xdecel': max_xdecel,
                'max_yawaccel': max_yawaccel,
                'max_yawdecel': max_yawdecel,
                'baud_rate': p2os_baud_rate,
            }],
            remappings=remappings
        )
    ]


def create_bno055_node(context, *args, **kwargs):
    """
    Creates the BNO055 IMU node with a dynamically resolved frame_id
    matching the mast URDF link name.
    """
    prefix = LaunchConfiguration('prefix').perform(context)
    imu_frame_id = f"{prefix}/mast/imu_link"

    config = os.path.join(
        get_package_share_directory('grunt_bringup'),
        'config',
        'bno055_params.yaml'
    )

    return [
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            parameters=[config, {'frame_id': imu_frame_id}],
        )
    ]

def create_arm_preset_nodes(context, *args, **kwargs):
    """
    Creates the arm preset publisher node and optional initial preset command.
    Handles namespace correctly when launched inside the parent namespace group.
    """
    prefix = LaunchConfiguration('prefix').perform(context)
    initial_preset = LaunchConfiguration('initial_arm_preset').perform(context)

    nodes = [
        Node(
            package='grunt_util',
            executable='arm_preset_publisher.py',
            name='arm_preset_publisher',
            parameters=[{'prefix': f'{prefix}/arm1/'}],
            output='screen',
        )
    ]

    if initial_preset:
        from launch.actions import TimerAction, ExecuteProcess
        nodes.append(
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'topic', 'pub', '--once',
                            f'/{prefix}/arm_preset',
                            'std_msgs/msg/String',
                            f"{{data: '{initial_preset}'}}"
                        ],
                        output='screen'
                    )
                ]
            )
        )

    return nodes


def generate_launch_description():

    # Declare arguments for prefix (namespace) and other parameters
    namespace_arg = DeclareLaunchArgument('prefix', default_value='mybot1', description='Namespace for all nodes. eg. mybot')
    group_arg = DeclareLaunchArgument('group', default_value='', description='Group name for all nodes. eg. chassis')

    # Construct the path to the config directory
    config_directory = os.path.join(get_package_share_directory('ublox_gps'), 'config')

    # Declare the launch arguments
    launch_arguments = [
        namespace_arg,
        group_arg,
        DeclareLaunchArgument('P2OS_Driver', default_value='1', description='Start the P3AT driver node'),
        DeclareLaunchArgument('P3AT_port', default_value='/dev/grunt_p3at', description='Port for the serial connection to the P3AT Arcos controller'),
        DeclareLaunchArgument('use_sonar', default_value='False', description='Enable or disable the sonar array'),
        DeclareLaunchArgument('enable_motors', default_value='1', description='enable motors on start'),
        DeclareLaunchArgument('DescribeChassisOnly', default_value='0', description='Publish urdf for chassis'), # choose this to publish chassis description
        DeclareLaunchArgument('DescribeWithArm', default_value='1', description='Publish Chassis+Arm and Start Arm'), #  or this to publish chassis + arm and start arm 
        DeclareLaunchArgument('Lidar', default_value='0', description='Enable RPLidar'),
        DeclareLaunchArgument('Arm', default_value='0', description='Start the RoArm'),
        DeclareLaunchArgument('ArmGUI', default_value='0', description='Start joint_state_publisher_gui for arm control'),
        DeclareLaunchArgument('ArmPreset', default_value='1', description='Start arm preset publisher for named positions'),
        DeclareLaunchArgument('initial_arm_preset', default_value='bumper', description='Initial arm preset on startup (e.g., bumper, tenhut, lookout@forward)'),
        DeclareLaunchArgument('RTK', default_value='1', description='Enable GNSS RTK'),
        DeclareLaunchArgument('IMU', default_value='1', description='Enable BNO055 IMU'),
        DeclareLaunchArgument('Localization', default_value='0', description='Enable robot_localization EKF stack (requires IMU and RTK)'),
        DeclareLaunchArgument('gps_quality_threshold', default_value='4', description='Min GPS quality for heading cal (0=none..5=rtk_fixed)'),
        DeclareLaunchArgument('gps_h_acc_cal_threshold_m', default_value='0.5', description='Max h_acc (m) for heading calibration'),
        DeclareLaunchArgument('Nav2', default_value='0', description='Enable Nav2 GPS waypoint following (requires Localization)'),
        DeclareLaunchArgument('KeyboardTeleop', default_value='0', description='Start keyboard driven teleop'),
        DeclareLaunchArgument('JoystickTeleop', default_value='1', description='Start joystick driven teleop'),
        # P2OS velocity and acceleration limits
        # Firmware ceilings (values above these are silently clamped by ARCOS):
        #   transveltop=1.5 m/s, rotveltop=6.28 rad/s (360 deg/s)
        #   transacctop=2.0 m/s² (applies to both accel and decel)
        #   rotacctop=5.24 rad/s² / 300 deg/s² (applies to both accel and decel)
        DeclareLaunchArgument('max_xspeed', default_value='0.5', description='Max translational velocity (m/s). Firmware ceiling: 1.5 m/s'),
        DeclareLaunchArgument('max_yawspeed', default_value='1.7453', description='Max rotational velocity (rad/s). Firmware default: ~1.75 rad/s (100 deg/s). Ceiling: 6.28 rad/s (360 deg/s)'),
        DeclareLaunchArgument('max_xaccel', default_value='0.0', description='Translational acceleration (m/s²). 0.0 = firmware default. Ceiling: 2.0 m/s²'),
        DeclareLaunchArgument('max_xdecel', default_value='0.0', description='Translational deceleration (m/s²). 0.0 = firmware default. Ceiling: 2.0 m/s²'),
        DeclareLaunchArgument('max_yawaccel', default_value='0.0', description='Rotational acceleration (rad/s²). 0.0 = firmware default (~1.75 rad/s²). Ceiling: 5.24 rad/s²'),
        DeclareLaunchArgument('max_yawdecel', default_value='0.0', description='Rotational deceleration (rad/s²). 0.0 = firmware default (~1.75 rad/s²). Ceiling: 5.24 rad/s²'),
        DeclareLaunchArgument('p2os_baud_rate', default_value='0', description='Serial baud rate (9600/19200/38400/57600/115200). 0 = use robot model default'),
        # Teleop velocity limits (what joystick sends before driver clamp)
        DeclareLaunchArgument('max_vx', default_value='0.6', description='Teleop max linear velocity (m/s)'),
        DeclareLaunchArgument('max_vx_turbo', default_value='0.6', description='Teleop max linear velocity with turbo button (m/s)'),
        DeclareLaunchArgument('max_vw', default_value='0.8', description='Teleop max angular velocity (rad/s)'),
        DeclareLaunchArgument('max_vw_turbo', default_value='0.8', description='Teleop max angular velocity with turbo button (rad/s)'),
        DeclareLaunchArgument('teleop_rate', default_value='10', description='Teleop publish loop rate (Hz)'),
    ]

    # Define the included launch descriptions with conditions
    # P2OS driver node with dynamic frame ID configuration
    gp_p2os = GroupAction(
        actions=[
            OpaqueFunction(function=create_p2os_node)
        ],
        condition=IfCondition(LaunchConfiguration('P2OS_Driver'))
    )
    gp_key_tele = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('p2os_bringup'), '/launch', '/teleop_keyboard_launch.py'
                ])
            )
        ],
        condition=IfCondition(LaunchConfiguration('KeyboardTeleop'))
    )
    gp_joy_tele = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('grunt_bringup'), '/launch', '/teleop_joy_launch.py'
                ]),
                launch_arguments={
                    'max_vx': LaunchConfiguration('max_vx'),
                    'max_vx_turbo': LaunchConfiguration('max_vx_turbo'),
                    'max_vw': LaunchConfiguration('max_vw'),
                    'max_vw_turbo': LaunchConfiguration('max_vw_turbo'),
                    'teleop_rate': LaunchConfiguration('teleop_rate'),
                }.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration('JoystickTeleop'))
    )

    # twist_mux: multiplexes velocity sources with priority and e-stop locks
    twist_mux_config = os.path.join(get_package_share_directory('grunt_bringup'), 'config', 'twist_mux.yaml')
    gp_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[('cmd_vel_out', 'cmd_vel')],
    )
    gp_lidar = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('grunt_bringup'), '/launch', '/rplidar_launch.py'
                ])
            )
        ],
        condition=IfCondition(LaunchConfiguration('Lidar'))
    )
    gp_motors_on = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('p2os_bringup'), '/launch', '/enable_motors_launch.py'
                ]),
                # Supply the namespace applied below, which is otherwise not honored by this launch command
                launch_arguments={
                    'prefix':LaunchConfiguration('prefix')
                }.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration('enable_motors'))
    )
    gp_description = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('p2os_urdf'), '/launch', '/pioneer3at_urdf_launch.py'
                ]),
                # Supply the namespace applied below, which is otherwise not honored by this launch command
                launch_arguments={
                    'prefix':LaunchConfiguration('prefix')
                }.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration('DescribeChassisOnly'))
    )
    
    gp_description_arm = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('grunt_bringup'), '/launch', '/describe.with.arm.py'
                ]),
                # Alert that there is a namespace applied
                launch_arguments={
                    'namespace':LaunchConfiguration('prefix'),
                    'use_gui': LaunchConfiguration('ArmGUI'),
                }.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration('DescribeWithArm'))
    )

    gp_imu = GroupAction(
        actions=[
            OpaqueFunction(function=create_bno055_node)
        ],
        condition=IfCondition(LaunchConfiguration('IMU'))
    )

    gp_localization = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('grunt_bringup'), '/launch', '/localization.launch.py'
                ]),
                launch_arguments={
                    'prefix': LaunchConfiguration('prefix'),
                    'gps_quality_threshold': LaunchConfiguration('gps_quality_threshold'),
                    'gps_h_acc_cal_threshold_m': LaunchConfiguration('gps_h_acc_cal_threshold_m'),
                }.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration('Localization'))
    )

    gp_nav2 = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('grunt_bringup'), '/launch', '/nav2.launch.py'
                ]),
                launch_arguments={
                    'prefix': LaunchConfiguration('prefix'),
                }.items()
            ),
            # Joystick e-stop: toggle button locks all velocity via twist_mux
            # and cancels any active Nav2 mission
            Node(
                package='grunt_bringup',
                executable='joy_estop',
                name='joy_estop',
                parameters=[{'estop_button': 1}],  # B button (red, GameSir Nova Lite)
                # Crawl moved from B to dpad, so B is now free for e-stop.
                output='screen',
            ),
        ],
        condition=IfCondition(LaunchConfiguration('Nav2'))
    )

    gp_arm_preset = GroupAction(
        actions=[
            OpaqueFunction(function=create_arm_preset_nodes)
        ],
        condition=IfCondition(LaunchConfiguration('ArmPreset'))
    )

    gp_description_rtk = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('grunt_bringup'), '/launch', '/gps.rtk.launch.py'
                ]),
                # Alert that there is a namespace applied, so we don't duplicate it in the target launch file
                launch_arguments={
                    'prefix':LaunchConfiguration('prefix'),
                    'group':'rtk' #re-assert group since we'd set it to ""
                }.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration('RTK'))
    )


    # Parent GroupAction that applies a namespace
    parent_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('prefix')),
        PushRosNamespace(LaunchConfiguration('group')),
        gp_p2os,
        gp_twist_mux,
        gp_description,
        gp_description_arm,
        gp_arm_preset,
        gp_description_rtk,
        gp_imu,
        gp_localization,
        gp_nav2,
        gp_lidar,
        gp_joy_tele,
        gp_key_tele,
        gp_motors_on
    ])

    # Launch description
    return LaunchDescription(launch_arguments + [parent_group])

if __name__ == '__main__':
    generate_launch_description()
