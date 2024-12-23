import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xml.etree.ElementTree as ET


def generate_robot_description(prefix, subsystem, base_link_name):
    """
    Processes and merges two Xacro files into a single URDF string.
    
    Args:
        prefix (str): Namespace prefix for the robot's links and joints.
        subsystem (str): Name of the subsystem (/arm)
        base_link_name (str): Name of the robot's base link. (/mybot1)
    
    Returns:
        str: Merged URDF content as a string.
    
    Raises:
        RuntimeError: If Xacro processing fails.
    """
    # Paths to Xacro files
    chassis_xacro_file = os.path.join(
        get_package_share_directory('p2os_urdf'),
        'defs',
        'pioneer3at.xacro'
    )

    arm_xacro_file = os.path.join(
        get_package_share_directory('roarm_description'),
        'urdf',
        'roarm_urdf_builder.xacro'
    )

    try:
        # Process the arm Xacro file
        arm_description_content = subprocess.check_output(
            ['xacro', arm_xacro_file, f'prefix:={prefix}{subsystem}/', f'base_link_name:={base_link_name}']
        ).decode('utf-8')

        # Process the chassis Xacro file
        chassis_description_content = subprocess.check_output(
            ['xacro', chassis_xacro_file, f'prefix:={prefix}/chassis/']
        ).decode('utf-8')

        # Parse the URDF contents into XML element trees
        chassis_root = ET.fromstring(chassis_description_content)
        arm_root = ET.fromstring(arm_description_content)

        # Dummy root to hold and connect the merged descriptions
        dummy_root_content = f"""
        <dummy>
        <link name="{prefix}/base_link"/>
        <joint name="base_to_chassis" type="fixed">
            <parent link="{prefix}/base_link"/>
            <child link="{prefix}/chassis/base_link"/>
        </joint>
        <joint name="chassis_to_arm" type="fixed">
            <origin rpy="0 0 0" xyz="0.205 0 0.0"/>
            <parent link="{prefix}/chassis/top_plate"/>
            <child link="{prefix}{subsystem}/base_link"/>
        </joint>
        </dummy>
        """

        print(dummy_root_content)
        dummy_root = ET.fromstring(dummy_root_content)
        # Now 'dummy_root' is a single element containing all new elements inside.
        # Get its children (link and joint elements)
        tree_root =  list(dummy_root)
       
        # Merge the descriptions
        # Create a new <robot> tag as the true root
        merged_robot = ET.Element("robot", {"name": "merged_robot"})
        
        for elem in tree_root:
            merged_robot.append(elem)

        for child in chassis_root:
            merged_robot.append(child)

        for child in arm_root:
            merged_robot.append(child)

        # Convert the merged URDF back to string
        return ET.tostring(merged_robot, encoding='unicode')
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Error processing Xacro files: {e}")


def launch_setup(context, *args, **kwargs):
    """
    Sets up the launch configuration by resolving parameters and creating nodes.
    
    Args:
        context: Launch context.
    
    Returns:
        list: List of Node actions to be added to the launch description.
    """
    # Resolve launch arguments
    # If we have inherited a namespace from a calling context, we'll use that as our prefix for descriptions and not apply further namespaces on nodes or parameters
    inherited_namespace = LaunchConfiguration('namespace').perform(context)
    
    if  inherited_namespace:
        print(f'inherited namespace:{inherited_namespace}')
        prefix = f'/{inherited_namespace}'
        apply_namespace=prefix
        
    else: #prefix and namespace come from prefix launch argument
        print(f'NO inherited namespace:{inherited_namespace}')
        apply_namespace = LaunchConfiguration('prefix').perform(context)
        prefix = apply_namespace
    subsystem = LaunchConfiguration('subsystem_name').perform(context)
    base_link_name = LaunchConfiguration('base_link_name').perform(context)

    # Generate the merged robot_description
    robot_description_content = generate_robot_description(prefix, subsystem, base_link_name)
    
    #print(robot_description_content)
    
    # Create robot_description parameter
    robot_description = {'robot_description': robot_description_content}

    # Define parent_frame_id for static transform
    parent_frame_id = f"{prefix}{subsystem}/top_plate"

    # Define child_frame_ids for static transforms
    child_frame_id = f"{prefix}/chassis/base_link"
    arm_frame_id = f"{prefix}{subsystem}/{base_link_name}"

    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=apply_namespace,
        output='screen',
        parameters=[robot_description],
    )

    # Create joint_state_publisher_gui node
    arm_joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=f"{apply_namespace}",
        output='screen',
        parameters=[robot_description],
    )

    # Create roarm_driver node
    roarm_driver_node = Node(
        package='roarm_driver',
        executable='roarm_driver',
        name='arm_driver',
        #namespace='/arm1',
        namespace=f"{apply_namespace}{subsystem}",
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port').perform(context),
            'baud_rate': 115200,
        }],
        #remappings=[(f'/mybot1/arm1/joint_states', f'/mybot1/joint_states')]
        remappings=[(f'{apply_namespace}{subsystem}/joint_states', f'{apply_namespace}/joint_states')]
    )

    return [
        robot_state_publisher_node,
        arm_joint_state_publisher_node,
        roarm_driver_node
    ]


def generate_launch_description():
    """
    Generates the launch description with declared arguments and actions.
    
    Returns:
        LaunchDescription: The complete launch description.
    """
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/grunt_arm',
        description='Serial port to be used by the node'
    )

    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='/mybot1',
        description='Prefix for the arm links and joints'
    )
    
    subsystem_name_arg = DeclareLaunchArgument(
        'subsystem_name',
        default_value='/arm1',
        description="Name of the subsystem"
    )

    base_link_name_arg = DeclareLaunchArgument(
        'base_link_name',
        default_value='base_link',
        description="Name of the arm's base link"
    )

    # Use OpaqueFunction to dynamically create nodes with resolved parameters
    setup_nodes = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        serial_port_arg,
        prefix_arg,
        subsystem_name_arg,
        base_link_name_arg,
        setup_nodes
    ])
