import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xml.etree.ElementTree as ET

def generate_launch_description():
    # Declare the launch argument for serial port
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/grunt_arm',
        description='Serial port to be used by the node'
    )

    # Declare prefix for namespace and tf tree elements
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='/mybot1/arm1/',
        description='Prefix for the arm links and joints'
    )

    base_link_name_arg = DeclareLaunchArgument(
        'base_link_name',
        default_value='base_link',
        description="Name of the arm's base link"
    )

    end_rot_arg = DeclareLaunchArgument(
        'end_rot',
        default_value='0',
        description="CCW rotation of the arm's tool (look at it), can be 0, 90, 180, 270"
    )

    # Get the launch configuration variables
    prefix = LaunchConfiguration('prefix')
    base_link_name = LaunchConfiguration('base_link_name')
    end_rot = LaunchConfiguration('end_rot')

    # Define the path to the builder Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('roarm_description'),
        'urdf',
        'roarm_urdf_builder.xacro'
    )

    # Process the Xacro file to generate the robot description parameter
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' prefix:=', prefix,
        ' base_link_name:=', base_link_name,
        ' end_rot:=', end_rot,
    ])

    # Wrap robot_description_content with ParameterValue
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Construct the child_frame_id by concatenating prefix and base_link_name using PythonExpression
    child_frame_id = PythonExpression(["'", prefix, "' + '", base_link_name, "'"])

    # Start the robot_state_publisher node for the arm
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=[prefix],
        output='screen',
        parameters=[robot_description],
        #arguments=['--ros-args', '--log-level', 'debug'], #if debugging needed
    )

    # Define the static transform publisher to link 'top_plate' to the arm's base link
    # Adjust the translation (x, y, z) and rotation (roll, pitch, yaw) as needed
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        namespace=[prefix],
        output='screen',
        arguments=[
            # Translation
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            # Rotation (Euler angles)
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            # Frames
            '--frame-id', 'top_plate',
            '--child-frame-id', child_frame_id
        ]
    )
    
    # some kind of joint state publisher is required for the arm joints or the 
    # tf tree won't be complete
    # here using the gui tool
    # otherwise joint states would be provided by the package used to control the arm
    arm_joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=[prefix],
        parameters=[robot_description],
        )
    
    # Declare the node with the serial port parameter
    roarm_driver_node = Node(
        package='roarm_driver',
        executable='roarm_driver',
        name='arm_driver',
        namespace=[prefix],
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        prefix_arg,
        base_link_name_arg,
        end_rot_arg,
        robot_state_publisher_node,
        static_tf_node,
        arm_joint_state_publisher_node,
        roarm_driver_node

    ])
