from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "grunt1/arm1/cam_sim_bottom_screw_frame", "grunt1/arm1/cam_live_base"]
        )
    ])
