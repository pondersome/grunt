"""Bag loading — parse the topics we care about into structured Python.

Output format: a dict keyed by short topic name (strips the `/grunt1/` prefix
by default), with each value being a list of tuples whose schema depends on
the topic type. See `TOPIC_SCHEMA` for the per-topic tuple layout.

Choosing tuples (not dataclasses or numpy arrays) for now — keeps deps light
and metric authors free to slice without ceremony. If a metric becomes
hot-path, that metric can promote its own data to numpy.

Adding a new topic:
    1. Add it to TOPIC_SCHEMA with a tuple description.
    2. Add an elif branch in `_decode_message`.
    3. Update the README's "planned extensions" table.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable, Optional

from mcap_ros2.reader import read_ros2_messages

from .geo import quat_yaw

# Topics we know how to parse. Other topics in a bag are silently ignored.
# Tuple schemas — what each parsed message gets reduced to:
TOPIC_SCHEMA = {
    "/grunt1/odometry/global":   "(t, x, y, yaw_rad)",
    "/grunt1/odometry/local":    "(t, yaw_rad, vx, vyaw, x, y)",
    "/grunt1/odometry/gps_map":  "(t, x, y)",
    "/grunt1/odometry/gps":      "(t, x, y)",
    "/grunt1/rtk/fix":           "(t, lat, lon, alt)",
    "/grunt1/rtk/fix_gated":     "(t, lat, lon, alt)",
    "/grunt1/rtk/navpvt":        "(t, heading_deg, head_acc_deg, gspeed_mps, carrier, h_acc_m)",
    "/grunt1/rtk/heading_imu":   "(t, yaw_rad, head_acc_deg)",
    "/grunt1/bno055/imu":        "(t, yaw_rad, vyaw_radps)",
    "/grunt1/cmd_vel":           "(t, vx, vyaw)",
    "/grunt1/cmd_vel_nav":       "(t, vx, vyaw)",
    "/grunt1/cmd_vel_nav_raw":   "(t, vx, vyaw)",
    "/grunt1/cmd_vel_joy":       "(t, vx, vyaw)",
    "/grunt1/sonar/cloud":       "(t, n_points)",
    "/grunt1/nav/collision_monitor_state": "(t, action_type)",
    "/grunt1/localization/status": "(t, status_dict)",
    "/grunt1/pose":              "(t, vx, vyaw)",  # p2os wheel odometry twist
    # RPP observables — added 2026-05-01 once the bagger started
    # capturing /nav/-namespaced topics correctly.
    "/grunt1/nav/lookahead_point":          "(t, x, y)",   # the carrot
    "/grunt1/nav/is_rotating_to_heading":   "(t, flag)",   # bool: in-place rotation
    "/grunt1/nav/received_global_plan":     "(t, n_poses, length_m)",  # path summary
    "/grunt1/nav/local_plan":               "(t, n_poses, length_m)",
}


def _decode_message(topic: str, msg, t: float) -> Optional[tuple]:
    """Reduce a deserialized ROS message to a tuple per TOPIC_SCHEMA."""
    if topic in ("/grunt1/odometry/global",):
        p = msg.pose.pose
        return (t, p.position.x, p.position.y, quat_yaw(p.orientation))
    if topic == "/grunt1/odometry/local":
        # Position included for carrot-tracking (carrot is in odom frame).
        p = msg.pose.pose
        return (t, quat_yaw(p.orientation),
                msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                p.position.x, p.position.y)
    if topic in ("/grunt1/odometry/gps_map", "/grunt1/odometry/gps"):
        p = msg.pose.pose.position
        return (t, p.x, p.y)
    if topic in ("/grunt1/rtk/fix", "/grunt1/rtk/fix_gated"):
        return (t, msg.latitude, msg.longitude, msg.altitude)
    if topic == "/grunt1/rtk/navpvt":
        # ublox NavPVT scale factors and flag layout
        return (t,
                msg.heading * 1e-5,           # deg
                msg.head_acc * 1e-5,          # deg
                msg.g_speed * 1e-3,           # m/s
                (msg.flags & 0xC0) >> 6,      # carrier: 0=none, 1=float, 2=fixed
                msg.h_acc * 1e-3)             # m
    if topic == "/grunt1/rtk/heading_imu":
        # NavPVT-derived IMU msg from navpvt_to_imu — yaw_var on the diagonal
        var = msg.orientation_covariance[8]
        head_acc_deg = math.degrees(math.sqrt(var)) if var > 0 else float("nan")
        return (t, quat_yaw(msg.orientation), head_acc_deg)
    if topic == "/grunt1/bno055/imu":
        return (t, quat_yaw(msg.orientation), msg.angular_velocity.z)
    if topic in ("/grunt1/cmd_vel", "/grunt1/cmd_vel_nav",
                 "/grunt1/cmd_vel_nav_raw", "/grunt1/cmd_vel_joy",
                 "/grunt1/pose"):
        # /grunt1/pose is nav_msgs/Odometry — twist matches Twist layout
        if topic == "/grunt1/pose":
            return (t, msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        return (t, msg.linear.x, msg.angular.z)
    if topic == "/grunt1/sonar/cloud":
        return (t, int(msg.width))
    if topic == "/grunt1/nav/collision_monitor_state":
        return (t, int(msg.action_type))
    if topic == "/grunt1/localization/status":
        try:
            return (t, json.loads(msg.data))
        except Exception:
            return None
    if topic == "/grunt1/nav/lookahead_point":
        # geometry_msgs/PointStamped — the carrot
        return (t, msg.point.x, msg.point.y)
    if topic == "/grunt1/nav/is_rotating_to_heading":
        return (t, bool(msg.data))
    if topic in ("/grunt1/nav/received_global_plan", "/grunt1/nav/local_plan"):
        # nav_msgs/Path — reduce to (count, total_arc_length)
        n = len(msg.poses)
        if n < 2:
            return (t, n, 0.0)
        L = 0.0
        for i in range(1, n):
            a = msg.poses[i-1].pose.position
            b = msg.poses[i].pose.position
            L += math.hypot(b.x - a.x, b.y - a.y)
        return (t, n, L)
    return None


def discover_mcap(bag_dir: str | Path) -> Path:
    """Find the .mcap file inside a `ros2 bag record` directory."""
    p = Path(bag_dir)
    if p.is_file() and p.suffix == ".mcap":
        return p
    candidates = sorted(p.glob("*.mcap"))
    if not candidates:
        raise FileNotFoundError(f"No .mcap file in {p}")
    return candidates[0]


def load_bag(bag_dir: str | Path,
             topics: Optional[Iterable[str]] = None) -> dict[str, list[tuple]]:
    """Parse a bag into a dict-of-lists.

    Args:
        bag_dir: directory or .mcap file path.
        topics: optional iterable of topic names to keep. None = all known.

    Returns:
        dict mapping topic name → list of tuples per `TOPIC_SCHEMA`.
        Topics not present in the bag come back as empty lists.
    """
    mcap = discover_mcap(bag_dir)
    keep = set(topics) if topics else set(TOPIC_SCHEMA.keys())
    out = {t: [] for t in keep}
    for m in read_ros2_messages(mcap, topics=sorted(keep)):
        topic = m.channel.topic
        if topic not in keep:
            continue
        t = m.publish_time_ns / 1e9
        decoded = _decode_message(topic, m.ros_msg, t)
        if decoded is not None:
            out[topic].append(decoded)
    return out


def time_origin(parsed: dict[str, list[tuple]]) -> float:
    """First sample time across all topics — useful for converting to t_rel."""
    candidates = []
    for samples in parsed.values():
        if samples:
            candidates.append(samples[0][0])
    if not candidates:
        raise ValueError("No samples in any topic — empty bag?")
    return min(candidates)
