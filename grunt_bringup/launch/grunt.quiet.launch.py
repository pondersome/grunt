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
    Creates the p2os_driver node with dynamically resolved frame IDs.
    """
    prefix = LaunchConfiguration('prefix').perform(context)
    use_sonar = LaunchConfiguration('use_sonar').perform(context)
    p3at_port = LaunchConfiguration('P3AT_port').perform(context)

    # Construct frame IDs with namespace prefix
    base_link_frame_id = f"{prefix}/base_link"
    odom_frame_id = f"{prefix}/odom"

    return [
        Node(
            package='p2os_driver',
            executable='p2os_driver',
            name='p2os_driver',
            parameters=[{
                'use_sonar': use_sonar == 'True' or use_sonar == 'true',
                'port': p3at_port,
                'base_link_frame_id': base_link_frame_id,
                'odom_frame_id': odom_frame_id
            }],
            remappings=[('pose','odom')]
        )
    ]

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
        DeclareLaunchArgument('RTK', default_value='1', description='Enable GNSS RTK'),
        DeclareLaunchArgument('KeyboardTeleop', default_value='0', description='Start keyboard driven teleop'),
        DeclareLaunchArgument('JoystickTeleop', default_value='1', description='Start joystick driven teleop')
        
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
                    FindPackageShare('p2os_bringup'), '/launch', '/teleop_joy_launch.py'
                ])
            )
        ],
        condition=IfCondition(LaunchConfiguration('JoystickTeleop'))
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
                    'namespace':LaunchConfiguration('prefix')
                }.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration('DescribeWithArm'))
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
        gp_description,
        gp_description_arm,
        gp_description_rtk,
        gp_lidar,
        gp_joy_tele,
        gp_key_tele,
        gp_motors_on
    ])

    # Launch description
    return LaunchDescription(launch_arguments + [parent_group])

if __name__ == '__main__':
    generate_launch_description()
