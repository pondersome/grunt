"""Launch markgol's l2lidar_node with Grunt-side overrides.

Conventions match grunt_bringup/launch/gps.rtk.launch.py:
  - `prefix` arg = robot namespace (e.g. "grunt1"). Empty default.
  - `group` arg = subsystem namespace under prefix (default "l2"). Lets two
    L2 instances coexist on one robot if it ever comes up.
  - `push_prefix` arg = whether THIS launch should push `prefix` as a
    namespace. Default true (standalone). The parent grunt.quiet.launch.py
    passes false because its parent_group already pushed prefix; pushing
    again here would yield /<prefix>/<prefix>/l2/... topics.
  - Frame IDs are built as [prefix, '/mast/...'] regardless of push_prefix,
    because TF frame names aren't subject to PushRosNamespace.

Topics published (assuming defaults):
  /<prefix>/<group>/points    PointCloud2  ~5.5 Hz
  /<prefix>/<group>/imu       Imu          ~1 kHz
  /tf_static                   l2 → l2_imu (the documented intrinsic L2 IMU offset)

TF chain end-to-end (with URDF), V0.5 single-frame topology. Grunt keeps the
legacy `l2` frame name (no `_link` suffix) for downstream-consumer continuity
— bag analyses, rviz/Foxglove layouts, Nav2 layer configs all reference the
existing names. The IMU frame auto-derives as `l2_imu` (no `_link` suffix to
strip on the cloud_frame, so the node just appends `_imu`).

  <prefix>/mast/base_link
    → <prefix>/mast/l2          (URDF: rpy=(pi, -pi/2, 0), xyz=(0.025, 0, 0.34))
    → <prefix>/mast/l2_imu      (node: intrinsic L2 IMU offset)

Run examples:
  Standalone:    ros2 launch grunt_bringup grunt_l2.launch.py prefix:=grunt1
  Included:      (push_prefix:=false passed by parent)
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


CONFIG = os.path.join(
    get_package_share_directory("grunt_bringup"),
    "config",
    "grunt_l2_params.yaml",
)


def _build_group(context):
    prefix = LaunchConfiguration("prefix")
    push_prefix = LaunchConfiguration("push_prefix").perform(context).lower() in (
        "1", "true", "yes")

    inner = []
    if push_prefix:
        inner.append(PushRosNamespace(prefix))
    inner.append(PushRosNamespace(LaunchConfiguration("group")))

    inner.append(Node(
        package="l2lidar_node",
        executable="l2lidar_node",
        name="l2lidar_node",
        output="screen",
        parameters=[
            CONFIG,
            {
                # V0.5 single-frame topology. cloud_frame points to the
                # URDF-declared mounting link (mast.xacro). The driver
                # auto-derives imu_frame from cloud_frame — since our
                # name has no `_link` suffix, it just appends `_imu` to
                # produce <prefix>/mast/l2_imu. URDF owns the extrinsic
                # placement; the driver emits only the intrinsic
                # cloud → imu static TF.
                #
                # Grunt keeps the legacy `l2` frame name (rather than
                # the upstream `l2lidar_link` default) for downstream-
                # consumer continuity: existing bags, rviz/Foxglove
                # layouts, and Nav2 layer configs all reference
                # `grunt1/mast/l2`. Renaming would force a sweep across
                # all of those.
                "cloud_frame": [prefix, "/mast/l2"],
                "publish_tf": True,
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
    ))

    # Self-return filter: drops points inside a static cuboid
    # around the robot so the L2 cloud fed to the Nav2 costmap
    # does not mark the chassis/mast/arm itself. Republishes
    # points_filtered in the original mast/l2 frame.
    inner.append(Node(
        package="grunt_bringup",
        executable="l2_self_filter",
        name="l2_self_filter",
        output="screen",
        parameters=[{
            "input_topic": "points",
            "output_topic": "points_filtered",
            "base_frame": [prefix, "/base_link"],
            # Calibrated 2026-05-21 from l2_parked_ground by
            # grunt_analysis.self_filter_cal (fit to actual
            # self-returns + 10 cm margin). Replaces the loose
            # hand-set box whose 1.10 m forward reach blanked
            # real obstacles until ~1 m ahead. z_max is the
            # ten-hut arm envelope; mast/antenna returns above
            # the costmap's max_obstacle_height (1.5 m) need no
            # masking. Re-run the tool if the robot's shape
            # changes (arm pose, collision damage, sensors).
            "cuboid_x_min": -0.13, "cuboid_x_max": 0.80,
            "cuboid_y_min": -0.48, "cuboid_y_max": 0.37,
            "cuboid_z_min":  0.17, "cuboid_z_max": 1.08,
        }],
        respawn=True,
        respawn_delay=3.0,
    ))

    # Ground-plane removal: drops drivable-ground points so the
    # Nav2 voxel_layer marks only real obstacles, not sloped
    # ground. Consumes points_filtered, emits points_obstacles.
    inner.append(Node(
        package="grunt_bringup",
        executable="l2_ground_filter",
        name="l2_ground_filter",
        output="screen",
        parameters=[{
            "input_topic": "points_filtered",
            "output_topic": "points_obstacles",
            "base_frame": [prefix, "/base_link"],
        }],
        respawn=True,
        respawn_delay=3.0,
    ))

    return [GroupAction(inner)]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "prefix", default_value="",
            description="Robot namespace prefix, e.g. 'grunt1'. Empty for un-namespaced."),
        DeclareLaunchArgument(
            "group", default_value="l2",
            description="Per-subsystem namespace under prefix, e.g. 'l2'."),
        DeclareLaunchArgument(
            "push_prefix", default_value="true",
            description="If true, this launch pushes `prefix` as a namespace. "
                        "Set false when a parent launch has already pushed it."),
        OpaqueFunction(function=_build_group),
    ])
