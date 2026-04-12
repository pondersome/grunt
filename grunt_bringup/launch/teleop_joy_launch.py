from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define paths to package shares and config files
    #p2os_launch_dir = FindPackageShare('p2os_launch')
    #joystick_config = p2os_launch_dir.find('launch/logi_gamepad.yaml')

    return LaunchDescription([
        # Teleop velocity limits (m/s for linear, rad/s for angular)
        DeclareLaunchArgument('max_vx', default_value='0.6', description='Max linear velocity (m/s)'),
        DeclareLaunchArgument('max_vx_turbo', default_value='0.6', description='Max linear velocity when turbo button held (m/s)'),
        DeclareLaunchArgument('max_vw', default_value='0.8', description='Max angular velocity (rad/s)'),
        DeclareLaunchArgument('max_vw_turbo', default_value='0.8', description='Max angular velocity when turbo button held (rad/s)'),
        DeclareLaunchArgument('teleop_rate', default_value='10', description='Teleop publish loop rate (Hz)'),

        # Declare the parameters for teleop_base node
        Node(
            package='p2os_teleop',
            executable='p2os_teleop',
            name='p2os_teleop',
            # Stop publishing when deadman released (after 3 zero-flush messages).
            # Allows twist_mux to fall through to Nav2 commands (priority 50).
            # Without this, continuous zero-vel at priority 100 blocks Nav2.
            arguments=['--deadman_no_publish'],
            remappings=[
                ('/des_vel', '/base_controller/command'),
                ('cmd_vel', 'cmd_vel_joy'),  # route through twist_mux
            ],
            parameters=[
                {'axis_vx': 1},
                {'axis_vw': 2},
                {'axis_vy': 0},
                {'deadman_button': 6},         # left bumper (GameSir Nova Lite)
                {'turbo_button': 7},          # right bumper
                {'crawl_axis': 7},            # dpad vertical: up=fwd crawl, down=rev crawl
                {'crawl_button': -1},         # legacy button disabled (was B)
                {'reverse_crawl_button': -1}, # legacy button disabled (was A)
                {'max_vx': LaunchConfiguration('max_vx')},
                {'max_vx_turbo': LaunchConfiguration('max_vx_turbo')},
                {'max_vw': LaunchConfiguration('max_vw')},
                {'max_vw_turbo': LaunchConfiguration('max_vw_turbo')},
                {'teleop_rate': LaunchConfiguration('teleop_rate')},
            ]
        ),

        # Node for joystick control with device parameter from YAML
        Node(
            package='joy',
            executable='joy_node',
            name='joy_controller',
            #parameters=[joystick_config]
            parameters = [{'dev': '/dev/input/js0'}] #can override from command line, F310 is usually js1
        ),
    ])