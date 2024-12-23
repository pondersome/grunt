import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare launch arguments with default values
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='grunt_arm_',
        #default_value='',
        description='Prefix for the arm links and joints'
    )

    base_link_name_arg = DeclareLaunchArgument(
        'base_link_name',
        #default_value='arm_base_link',
        default_value='base_link',
        description="Name of the arm's base link"
    )

    # Get the launch configuration variables
    prefix = LaunchConfiguration('prefix')
    base_link_name = LaunchConfiguration('base_link_name')

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
        ' base_link_name:=', base_link_name
    ])

    # Wrap robot_description_content with ParameterValue
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Construct the child_frame_id by concatenating prefix and base_link_name using PythonExpression
    child_frame_id = PythonExpression(["'", prefix, "' + '", base_link_name, "'"])

    # Start the robot_state_publisher node for the arm
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        #arguments=['--ros-args', '--log-level', 'debug'], #if debugging needed
        remappings=[('/robot_description', '/arm_robot_description'),
                    ('/joint_states', '/arm/joint_states')]
    )

    # Define the static transform publisher to link 'top_plate' to the arm's base link
    # Adjust the translation (x, y, z) and rotation (roll, pitch, yaw) as needed
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arm_static_tf_pub',
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
    
    # some kind of joint state publisher is required
    # here using the gui tool
    # otherwise this would be provided by the package used to control the arm
    arm_joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='arm_joint_state_publisher_gui',
        parameters=[robot_description],
        remappings=[('/joint_states', '/arm/joint_states'),
                    ('/robot_description', '/arm_robot_description')]
        )

    return LaunchDescription([
        prefix_arg,
        base_link_name_arg,
        robot_state_publisher_node,
        static_tf_node,
        arm_joint_state_publisher_node

    ])
