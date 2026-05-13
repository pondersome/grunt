"""Shared helpers for the deeper-analysis modules (segdev, onsets,
planner_audit, costmap_render).

These take raw mcap streams (not the summarized output of `bag.load_bag`)
because they need full message contents: plan poses, sonar PointCloud2
data, OccupancyGrid raster. The summarizing decoders in `bag.py` drop
that detail to keep the metric pipeline fast.

If you find yourself adding a third or fourth consumer of one of these
helpers, that's the signal to move it into `bag.py` proper.
"""
from __future__ import annotations

import bisect
import math
import struct
from pathlib import Path
from typing import Iterator

import yaml

from .geo import GeoAnchor, lla_to_local_enu


# ──────────────────────────────────────────────────────────────────
# Geometry helpers — line/segment math reused across modules
# ──────────────────────────────────────────────────────────────────

def quat_yaw(q) -> float:
    """ROS quaternion → yaw in radians."""
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def signed_perp_to_seg(px: float, py: float,
                        ax: float, ay: float,
                        bx: float, by: float) -> float | None:
    """Signed perp distance from (px,py) to the INFINITE line through a→b.
    None if a==b. Positive = right of direction-of-travel a→b."""
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return None
    return ((px - ax)*dy - (py - ay)*dx) / L


def signed_offset_to_segment(px: float, py: float,
                              ax: float, ay: float,
                              bx: float, by: float) -> tuple[float | None, float | None]:
    """Signed perp distance + along-distance, only if projection lands
    within the segment's u ∈ [0,1] range. Returns (None, None) otherwise.
    """
    dx, dy = bx - ax, by - ay
    L2 = dx*dx + dy*dy
    if L2 < 1e-9:
        return None, None
    u = ((px - ax)*dx + (py - ay)*dy) / L2
    if u < 0 or u > 1:
        return None, None
    L = math.sqrt(L2)
    signed = ((px - ax)*dy - (py - ay)*dx) / L
    return signed, u * L


def closest_signed_offset(px: float, py: float,
                           segs: list) -> tuple[float, float, int]:
    """Find segment whose perpendicular projection of (px,py) gives the
    smallest |perp|. Returns (signed_offset, abs_offset, seg_idx).
    Returns (0.0, inf, -1) if no segment contains the projection.
    """
    best = (0.0, float("inf"), -1)
    for i, ((ax, ay), (bx, by)) in enumerate(segs):
        signed, _ = signed_offset_to_segment(px, py, ax, ay, bx, by)
        if signed is None:
            continue
        if abs(signed) < best[1]:
            best = (signed, abs(signed), i)
    return best


def find_traversed_segment(plan_poses: list[tuple[float, float]],
                            segs: list) -> int:
    """Determine which mission segment a plan is traversing.

    Strategy: count plan poses that project within u ∈ [-0.05, 1.05]
    of each segment AND lie within 3 m perpendicular. Segment with
    most matching poses wins. Returns -1 if none match.
    """
    best = (0, -1)
    for i, ((ax, ay), (bx, by)) in enumerate(segs):
        dx, dy = bx - ax, by - ay
        L2 = dx*dx + dy*dy
        if L2 < 1e-9:
            continue
        L = math.sqrt(L2)
        count = 0
        for (x, y) in plan_poses:
            u = ((x - ax)*dx + (y - ay)*dy) / L2
            if -0.05 <= u <= 1.05:
                perp = abs(((x - ax)*dy - (y - ay)*dx) / L)
                if perp <= 3.0:
                    count += 1
        if count > best[0]:
            best = (count, i)
    return best[1]


def cumulative_segment_length(segs: list) -> list[float]:
    """[0, len(seg0), len(seg0)+len(seg1), ...] for arc-length lookup."""
    cum = [0.0]
    for (a, b) in segs:
        cum.append(cum[-1] + math.hypot(b[0]-a[0], b[1]-a[1]))
    return cum


def wrap_pi(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ──────────────────────────────────────────────────────────────────
# Mission segments from mission YAML
# ──────────────────────────────────────────────────────────────────

def load_mission_segments(mission_yaml: Path,
                           anchor: GeoAnchor) -> tuple[list, list]:
    """Read a mission YAML, project waypoints to local ENU at the site
    anchor, return (wp_enu, segments). segments is [((ax,ay),(bx,by)), ...]
    in waypoint order."""
    with open(mission_yaml) as f:
        m = yaml.safe_load(f)
    wp_enu = [lla_to_local_enu(w["lat"], w["lon"], anchor)
              for w in m["waypoints"]]
    segs = list(zip(wp_enu[:-1], wp_enu[1:]))
    return wp_enu, segs


# ──────────────────────────────────────────────────────────────────
# PointCloud2 decoder (sonar)
# ──────────────────────────────────────────────────────────────────

def parse_pointcloud2(msg) -> list[tuple[float, float, float]]:
    """Decode sensor_msgs/PointCloud2 into (x, y, z) tuples.

    The grunt sonar cloud has 16 points max per cycle, all float32 in
    base_link frame. NaN points are dropped (sonar "no echo" sentinel).
    """
    if msg.point_step == 0 or msg.width == 0:
        return []
    offsets = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z")}
    if "x" not in offsets or "y" not in offsets:
        return []
    has_z = "z" in offsets
    data = msg.data
    if hasattr(data, "tobytes"):
        data = data.tobytes()
    pts = []
    n_pts = msg.width * msg.height
    for i in range(n_pts):
        base = i * msg.point_step
        x = struct.unpack_from("<f", data, base + offsets["x"])[0]
        y = struct.unpack_from("<f", data, base + offsets["y"])[0]
        if math.isnan(x) or math.isnan(y):
            continue
        z = (struct.unpack_from("<f", data, base + offsets["z"])[0]
             if has_z else 0.0)
        pts.append((x, y, z))
    return pts


# ──────────────────────────────────────────────────────────────────
# Plan-pose transform: base_link → map
# ──────────────────────────────────────────────────────────────────

def transform_plan_to_map(plan_poses_bl: list[tuple[float, float]],
                           frame_id: str,
                           plan_t: float,
                           odom: list,
                           odom_t: list[float],
                           max_skew_s: float = 0.3) -> list[tuple[float, float]] | None:
    """Transform a plan's poses to map frame using the closest odom pose
    at plan publish time.

    `odom` is [(t, x, y, yaw), ...] in TIME ORDER.
    `odom_t` is the precomputed [r[0] for r in odom] (for bisect).
    Returns None if no nearby odom sample, or pass-through if frame_id
    is already map.
    """
    if "base_link" not in frame_id:
        return list(plan_poses_bl)
    i = bisect.bisect_left(odom_t, plan_t)
    candidates = []
    if i < len(odom):
        candidates.append(odom[i])
    if i > 0:
        candidates.append(odom[i - 1])
    if not candidates:
        return None
    ref = min(candidates, key=lambda r: abs(r[0] - plan_t))
    if abs(ref[0] - plan_t) > max_skew_s:
        return None
    rx, ry, ryaw = ref[1], ref[2], ref[3]
    cy_, sy_ = math.cos(ryaw), math.sin(ryaw)
    return [(rx + px*cy_ - py*sy_, ry + px*sy_ + py*cy_)
            for (px, py) in plan_poses_bl]


# ──────────────────────────────────────────────────────────────────
# Fresh-onset detection (collision_monitor_state)
# ──────────────────────────────────────────────────────────────────

def fresh_onsets(coll_events: list[tuple[float, int]],
                  quiet_window_s: float = 10.0) -> list[tuple[float, int]]:
    """From a list of (t, action_type) collision_monitor events,
    return only the FRESH onsets: STATE goes 0 → non-zero with no
    non-zero events in the prior `quiet_window_s` seconds.

    Without this filter, a single foliage event becomes 10+ events as
    the robot extricates — clustering inflates any correlation count.
    """
    fresh = []
    last_nonzero = -1e9
    for (t, s) in coll_events:
        if s != 0:
            if t - last_nonzero >= quiet_window_s:
                fresh.append((t, s))
            last_nonzero = t
    return fresh


# ──────────────────────────────────────────────────────────────────
# Default site / mission fallback (matches cli.py convention)
# ──────────────────────────────────────────────────────────────────

def default_site_yaml(bag_dir: Path) -> Path | None:
    cand = Path.home() / "ros2_ws" / "grunt_missions" / "sites" / "ranchero" / "site.yaml"
    return cand if cand.exists() else None
