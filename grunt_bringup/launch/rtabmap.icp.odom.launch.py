from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

    # 1) Start ICP odometry on your cloud
    Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            #'frame_id':        'grunt1/arm1/cam_live_depth_optical_frame',  #do not use the sensor frame
            'frame_id':        'grunt1/base_link',  # robot base frame
            'odom_frame_id':   'odom',                              # what this node will broadcast
            'publish_tf':      True,                                # will broadcast odom→base_link
        }],
        remappings=[
            ('scan_cloud', '/grunt1/arm1/cam_live/depth/color/points'),
            # icp_odometry will publish nav_msgs/Odometry on "odom" and also
            # broadcast TF odom->base_link
        ]
    ),

    # 2) Launch RTAB‑Map, now *with* odometry input
    # Node(
    #     package='rtabmap_slam',
    #     executable='rtabmap',
    #     name='rtabmap',
    #     output='screen',
    #     parameters=[{
    #         'subscribe_scan_cloud': True,
    #         'subscribe_odom':       True,
    #         'frame_id':             'grunt1/base_link',
    #         'odom_frame_id':        'odom',        # must match the ICP node’s TF
    #         'publish_tf':           True,
    #         'RGBD/Enabled':         "false",
    #         'subscribe_odom':      True,
    #         'subscribe_rgb':        False,
    #         'subscribe_depth':      False,
    #         'subscribe_rgbd':       False,
    #         'approx_sync':          True,
    #     }],
    #     remappings=[
    #         ('scan_cloud', 'grunt1/arm1/cam_live/depth/color/points'),
    #         ('odom',       'odom'),
    #     ]
    # ),



    ])
