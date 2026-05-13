"""Launch markgol's l2lidar_node with Grunt-side overrides.

Conventions match grunt_bringup/launch/gps.rtk.launch.py:
  - `prefix` arg = robot namespace (e.g. "grunt1"). Empty default.
  - `group` arg = subsystem namespace under prefix (default "l2"). Lets two
    L2 instances coexist on one robot if it ever comes up.
  - Frame IDs are built as [prefix, '/mast/...'] to match the URDF's
    "$(arg prefix)<name>" convention (where the mast.xacro prefix is
    typically "<prefix>/mast/").

Topics published (assuming defaults):
  /<prefix>/<group>/points    PointCloud2  ~5.5 Hz
  /<prefix>/<group>/imu       Imu          ~1 kHz
  /tf_static                   l2_mount → l2 (identity, node-published)
                               l2 → l2_imu (the documented L2 IMU offset)

TF chain end-to-end (with URDF):
  <prefix>/mast/base_link
    → <prefix>/mast/l2_mount   (URDF: rpy=(pi, -pi/2, 0), xyz=(0.025, 0, 0.34))
    → <prefix>/mast/l2         (node: identity, robot_id → frame_id)
    → <prefix>/mast/l2_imu     (node: documented L2 IMU offset)

Run example:
  ros2 launch grunt_bringup grunt_l2.launch.py prefix:=grunt1
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


CONFIG = os.path.join(
    get_package_share_directory("grunt_bringup"),
    "config",
    "grunt_l2_params.yaml",
)


def generate_launch_description() -> LaunchDescription:
    prefix_arg = DeclareLaunchArgument(
        "prefix", default_value="",
        description="Robot namespace prefix, e.g. 'grunt1'. Empty for un-namespaced.")
    group_arg = DeclareLaunchArgument(
        "group", default_value="l2",
        description="Per-subsystem namespace under prefix, e.g. 'l2'.")

    prefix = LaunchConfiguration("prefix")

    return LaunchDescription([
        prefix_arg,
        group_arg,

        GroupAction([
            PushRosNamespace(prefix),
            PushRosNamespace(LaunchConfiguration("group")),

            Node(
                package="l2lidar_node",
                executable="l2lidar_node",
                name="l2lidar_node",
                output="screen",
                parameters=[
                    CONFIG,
                    {
                        # Match URDF's mast.xacro link names. Mast prefix is
                        # "<prefix>/mast/" by Grunt convention, so the L2's
                        # URDF link resolves to "<prefix>/mast/l2_mount".
                        "robot_id":      [prefix, "/mast/l2_mount"],
                        "frame_id":      [prefix, "/mast/l2"],
                        "imu_frame_id":  [prefix, "/mast/l2_imu"],
                        # robot_x/y/z stay at 0: the URDF carries the
                        # mount position+rotation; this node's auto-static
                        # TF (robot_id → frame_id) is the identity hop.
                        "robot_x": 0.0,
                        "robot_y": 0.0,
                        "robot_z": 0.0,
                    },
                ],
                remappings=[
                    # Markgol's node hardcodes absolute topic names; remap to
                    # relative so PushRosNamespace prefixes them correctly.
                    ("/points",   "points"),
                    ("/imu/data", "imu"),
                ],
                respawn=True,
                respawn_delay=3.0,
            ),
        ]),
    ])
