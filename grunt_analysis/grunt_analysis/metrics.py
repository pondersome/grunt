"""Composable metric functions.

Each metric takes:
  - `parsed`: output of `bag.load_bag`
  - `anchor`: a `GeoAnchor` for ENU conversion (only needed by GPS-aware metrics)

…and returns a result dict. The runner (`cli.py`) aggregates results into
a report. New metric = new function in this file (or a sibling module if
this gets large).

Convention: when a metric needs interpolation between two time series at
the same t, use `bisect.bisect_left` on a precomputed `_t` list. Cheap and
explicit.
"""
from __future__ import annotations

import bisect
import math
import statistics as s
from collections import Counter
from typing import Optional

from .geo import GeoAnchor, lla_to_local_enu


# ────────────────────────────────────────────────────────────────────
# Datum + EKF tracking
# ────────────────────────────────────────────────────────────────────

def datum_alignment(parsed, anchor: GeoAnchor) -> dict:
    """How well does navsat_transform's GPS-map odometry agree with raw GPS?

    Compares /grunt1/odometry/gps_map (supervisor relay of
    navsat_transform's /odometry/gps with frame_id=map) against
    flat-earth ENU(rtk/fix_gated). After /datum identity-orientation is
    sent, these should agree to within the gps_link → base_link mount
    offset (typically a few cm to tens of cm).
    """
    fix = parsed.get("/grunt1/rtk/fix_gated", [])
    gpm = parsed.get("/grunt1/odometry/gps_map", [])
    if not fix or not gpm:
        return {"name": "datum_alignment", "n": 0,
                "note": "missing fix_gated or gps_map"}
    fix_t = [r[0] for r in fix]
    diffs = []
    for (t, gx, gy) in gpm:
        i = bisect.bisect_left(fix_t, t)
        if i >= len(fix) or abs(fix[i][0] - t) > 0.5:
            continue
        fx, fy = lla_to_local_enu(fix[i][1], fix[i][2], anchor)
        diffs.append((gx - fx, gy - fy))
    if not diffs:
        return {"name": "datum_alignment", "n": 0,
                "note": "no time-aligned pairs"}
    dxs = [d[0] for d in diffs]
    dys = [d[1] for d in diffs]
    dists = [math.hypot(*d) for d in diffs]
    sd = sorted(dists)
    return {
        "name": "datum_alignment",
        "n": len(diffs),
        "median_dx_m": s.median(dxs),
        "median_dy_m": s.median(dys),
        "distance_median_m": s.median(dists),
        "distance_p90_m": sd[int(len(sd) * 0.9)] if sd else None,
        "distance_max_m": max(dists),
    }


def ekf_tracking(parsed, anchor: GeoAnchor) -> dict:
    """How well does ekf_global track the GPS truth?

    Compares /grunt1/odometry/global (EKF estimate) against flat-earth
    ENU(rtk/fix_gated). With a clean datum/sync chain, this should track
    GPS at the gps_link offset level.
    """
    odom = parsed.get("/grunt1/odometry/global", [])
    fix = parsed.get("/grunt1/rtk/fix_gated", [])
    if not odom or not fix:
        return {"name": "ekf_tracking", "n": 0,
                "note": "missing odometry/global or fix_gated"}
    fix_t = [r[0] for r in fix]
    samples = []  # (t_rel, distance)
    t0 = odom[0][0]
    for (t, ex, ey, _) in odom:
        i = bisect.bisect_left(fix_t, t)
        if i >= len(fix) or abs(fix[i][0] - t) > 0.5:
            continue
        fx, fy = lla_to_local_enu(fix[i][1], fix[i][2], anchor)
        samples.append((t - t0, math.hypot(ex - fx, ey - fy)))
    if not samples:
        return {"name": "ekf_tracking", "n": 0}
    dists = [d for _, d in samples]
    sd = sorted(dists)
    return {
        "name": "ekf_tracking",
        "n": len(samples),
        "median_m": s.median(dists),
        "p90_m": sd[int(len(sd) * 0.9)],
        "max_m": max(dists),
        # Time-sampled trajectory of distance — for plotting drift over time
        "trajectory": samples[:: max(1, len(samples) // 200)],
    }


# ────────────────────────────────────────────────────────────────────
# IMU + RTK health
# ────────────────────────────────────────────────────────────────────

def imu_health(parsed) -> dict:
    """BNO055 publish rate, gaps, and gyro_cal evolution (from supervisor status)."""
    bno = parsed.get("/grunt1/bno055/imu", [])
    status = parsed.get("/grunt1/localization/status", [])
    if not bno:
        return {"name": "imu_health", "rate_hz": 0, "note": "no IMU samples"}
    dur = bno[-1][0] - bno[0][0]
    rate = len(bno) / dur if dur > 0 else 0
    gaps = sum(1 for i in range(1, len(bno))
               if bno[i][0] - bno[i - 1][0] > 0.1)
    gcal_dist = Counter(
        d.get("imu_cal", {}).get("gyro") for _, d in status if d.get("imu_cal"))
    return {
        "name": "imu_health",
        "rate_hz": round(rate, 1),
        "gaps_over_100ms": gaps,
        "duration_s": round(dur, 1),
        "gyro_cal_distribution": dict(gcal_dist),
    }


def rtk_continuity(parsed) -> dict:
    """Carrier-phase distribution, h_acc, NONE-carrier lapses."""
    nav = parsed.get("/grunt1/rtk/navpvt", [])
    if not nav:
        return {"name": "rtk_continuity", "n": 0, "note": "no navpvt"}
    carriers = [c for _, _, _, _, c, _ in nav]
    h_accs = [ha for _, _, _, _, _, ha in nav]
    head_accs = [ha for _, _, ha, _, _, _ in nav]
    n = len(carriers)
    cc = Counter(carriers)
    sh = sorted(h_accs)
    # NONE-carrier lapse detection — carrier=0 spans
    t0 = nav[0][0]
    lapses = []
    cur = None
    cur_start = None
    for (t, _, _, _, c, _) in nav:
        if c == 0 and cur != 0:
            cur = 0; cur_start = t
        elif c != 0 and cur == 0:
            lapses.append((cur_start - t0, t - cur_start))
            cur = c
    return {
        "name": "rtk_continuity",
        "n": n,
        "carrier_dist": dict(cc),
        "fixed_pct": round(100 * cc.get(2, 0) / n, 1),
        "float_pct": round(100 * cc.get(1, 0) / n, 1),
        "none_pct": round(100 * cc.get(0, 0) / n, 1),
        "h_acc_median_m": s.median(h_accs),
        "h_acc_p90_m": sh[int(len(sh) * 0.9)],
        "h_acc_max_m": max(h_accs),
        "head_acc_median_deg": s.median(head_accs),
        "none_lapses": [(round(s, 1), round(d, 1)) for s, d in lapses],
    }


# ────────────────────────────────────────────────────────────────────
# Control + interventions
# ────────────────────────────────────────────────────────────────────

def cmd_vel_breakdown(parsed) -> dict:
    """Forward / reverse / stationary split + total velocity time fractions."""
    cmd = parsed.get("/grunt1/cmd_vel", [])
    if not cmd:
        return {"name": "cmd_vel_breakdown", "n": 0, "note": "no cmd_vel"}
    n = len(cmd)
    fwd = sum(1 for _, vx, _ in cmd if vx > 0.05)
    rev = sum(1 for _, vx, _ in cmd if vx < -0.05)
    z = n - fwd - rev
    return {
        "name": "cmd_vel_breakdown",
        "n": n,
        "forward_pct": round(100 * fwd / n, 1),
        "reverse_pct": round(100 * rev / n, 1),
        "zero_pct": round(100 * z / n, 1),
    }


def control_oscillation(parsed) -> dict:
    """S-turn signature: angular.z sign-flip rate during forward driving.

    Returns the implied closed-loop oscillation period if the chassis is
    visibly oscillating around the path. For a pure-pursuit controller
    this period typically equals (lookahead_time + actuation_lag).

    Also reports the cmd→actual yaw-rate correlation and ratio (a lag-
    accounting estimate of how much of the commanded omega the chassis
    actually delivers).
    """
    cmd = parsed.get("/grunt1/cmd_vel_nav_raw", [])
    odom_l = parsed.get("/grunt1/odometry/local", [])
    if not cmd:
        return {"name": "control_oscillation", "n": 0,
                "note": "no cmd_vel_nav_raw"}
    fwd_cmds = [(t, vz) for (t, vx, vz) in cmd if abs(vx) > 0.05]
    if len(fwd_cmds) < 10:
        return {"name": "control_oscillation", "n": len(fwd_cmds),
                "note": "insufficient forward driving"}
    azs = [v for _, v in fwd_cmds]
    flips = 0
    for i in range(1, len(fwd_cmds)):
        if (fwd_cmds[i][1] > 0.05) != (fwd_cmds[i - 1][1] > 0.05):
            flips += 1
    dur = fwd_cmds[-1][0] - fwd_cmds[0][0]
    flip_rate = flips / dur if dur > 0 else 0
    # cmd vs actual chassis vyaw — sample at ~200ms lag for actuation
    cmd_actual_pairs = []
    if odom_l:
        odom_t = [r[0] for r in odom_l]
        for (t, vz) in fwd_cmds[: len(fwd_cmds) // 4 * 3]:  # use middle-most chunk
            target = t + 0.2
            i = bisect.bisect_left(odom_t, target)
            if i < len(odom_l):
                cmd_actual_pairs.append((vz, odom_l[i][3]))
    corr = None
    ratio = None
    if cmd_actual_pairs and len(cmd_actual_pairs) > 30:
        try:
            corr = s.correlation([a for a, _ in cmd_actual_pairs],
                                  [b for _, b in cmd_actual_pairs])
        except Exception:
            corr = None
        ratios = [b / a for a, b in cmd_actual_pairs if abs(a) > 0.2]
        if ratios:
            ratio = s.median(ratios)
    return {
        "name": "control_oscillation",
        "n_forward_cmds": len(fwd_cmds),
        "abs_median_omega_radps": round(s.median([abs(x) for x in azs]), 3),
        "stdev_omega_radps": round(s.stdev(azs), 3) if len(azs) > 1 else 0,
        "sign_flips": flips,
        "flip_rate_hz": round(flip_rate, 3),
        "implied_period_s": round(1 / flip_rate, 2) if flip_rate > 0 else None,
        "cmd_actual_correlation": round(corr, 3) if corr is not None else None,
        "actual_over_cmd_ratio": round(ratio, 2) if ratio is not None else None,
    }


def crosstrack_error(parsed, waypoints_lla: list[tuple[float, float]],
                      anchor: GeoAnchor) -> dict:
    """Distance from EKF position to the nearest waypoint→waypoint segment.

    `waypoints_lla` is a list of (lat, lon) tuples in mission order.
    """
    odom = parsed.get("/grunt1/odometry/global", [])
    if not odom or len(waypoints_lla) < 2:
        return {"name": "crosstrack_error", "n": 0,
                "note": "missing odometry or waypoints"}
    wp_xy = [lla_to_local_enu(lat, lon, anchor) for (lat, lon) in waypoints_lla]

    def _seg_dist(p, a, b):
        ax, ay = a; bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-9:
            return math.hypot(p[0] - ax, p[1] - ay)
        t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))

    dists = []
    last_t = -1.0
    t0 = odom[0][0]
    for (t, x, y, _) in odom:
        if t - t0 - last_t < 0.5:
            continue
        last_t = t - t0
        d = min(_seg_dist((x, y), wp_xy[i], wp_xy[i + 1])
                for i in range(len(wp_xy) - 1))
        dists.append(d)
    if not dists:
        return {"name": "crosstrack_error", "n": 0}
    sd = sorted(dists)
    far = sum(1 for d in dists if d > 2.0)
    return {
        "name": "crosstrack_error",
        "n": len(dists),
        "median_m": s.median(dists),
        "p90_m": sd[int(len(sd) * 0.9)],
        "max_m": max(dists),
        "samples_over_2m_pct": round(100 * far / len(dists), 1),
    }


def interventions(parsed, min_burst_s: float = 0.5) -> dict:
    """Operator joy bursts: continuous non-zero cmd_vel_joy of >= min_burst_s.

    Each burst is reported as (t_rel_start, duration). t_rel relative to
    the start of /grunt1/cmd_vel_joy, NOT necessarily the bag start.
    """
    joy = parsed.get("/grunt1/cmd_vel_joy", [])
    if not joy:
        return {"name": "interventions", "n_bursts": 0}
    t0 = joy[0][0]
    bursts = []
    in_burst = False
    bs = None
    for (t, vx, vz) in joy:
        active = abs(vx) > 0.01 or abs(vz) > 0.01
        if active and not in_burst:
            bs = t
            in_burst = True
        elif not active and in_burst:
            if t - bs >= min_burst_s:
                bursts.append((round(bs - t0, 1), round(t - bs, 1)))
            in_burst = False
    return {
        "name": "interventions",
        "n_bursts": len(bursts),
        "bursts": bursts,  # list of (t_rel_start, duration)
    }


def rotation_mode_fraction(parsed) -> dict:
    """How much of the mission was spent in RPP's in-place rotation mode.

    Reads /grunt1/nav/is_rotating_to_heading (bool). High fraction
    suggests a path with a lot of sharp angles, or RPP's
    rotate_to_heading_min_angle threshold being too tight.
    """
    samples = parsed.get("/grunt1/nav/is_rotating_to_heading", [])
    if not samples:
        return {"name": "rotation_mode_fraction", "n": 0,
                "note": "no is_rotating_to_heading topic in bag"}
    if len(samples) < 2:
        return {"name": "rotation_mode_fraction", "n": len(samples),
                "note": "insufficient samples"}
    # Integrate time-in-rotating-state by treating each sample as
    # holding its value until the next sample.
    total_dur = samples[-1][0] - samples[0][0]
    rotating_dur = 0.0
    n_episodes = 0
    in_rot = False
    rot_start = None
    for i in range(1, len(samples)):
        if samples[i-1][1]:
            rotating_dur += samples[i][0] - samples[i-1][0]
        if samples[i-1][1] and not in_rot:
            in_rot = True
            rot_start = samples[i-1][0]
            n_episodes += 1
        elif not samples[i-1][1] and in_rot:
            in_rot = False
    return {
        "name": "rotation_mode_fraction",
        "n": len(samples),
        "rotating_pct": round(100 * rotating_dur / total_dur, 1) if total_dur else 0,
        "rotating_dur_s": round(rotating_dur, 1),
        "total_dur_s": round(total_dur, 1),
        "n_rotation_episodes": n_episodes,
    }


def carrot_tracking(parsed) -> dict:
    """Distance from EKF position to the carrot when each carrot is published.

    Reads /grunt1/nav/lookahead_point and /grunt1/odometry/global. The
    distance should be approximately `lookahead_dist` (or its
    velocity-scaled equivalent) when RPP is operating normally; large
    deviations indicate something wrong with the carrot computation
    or the path being received.
    """
    carrots = parsed.get("/grunt1/nav/lookahead_point", [])
    odom = parsed.get("/grunt1/odometry/global", [])
    if not carrots or not odom:
        return {"name": "carrot_tracking", "n": 0,
                "note": "missing carrot or odometry"}
    odom_t = [r[0] for r in odom]
    dists = []
    for (t, cx, cy) in carrots:
        i = bisect.bisect_left(odom_t, t)
        if i >= len(odom) or abs(odom[i][0] - t) > 0.5:
            continue
        ex, ey = odom[i][1], odom[i][2]
        dists.append(math.hypot(cx - ex, cy - ey))
    if not dists:
        return {"name": "carrot_tracking", "n": 0}
    sd = sorted(dists)
    return {
        "name": "carrot_tracking",
        "n": len(dists),
        "median_dist_to_carrot_m": round(s.median(dists), 3),
        "p10_dist_m": round(sd[len(sd) // 10], 3),
        "p90_dist_m": round(sd[int(len(sd) * 0.9)], 3),
        "stdev_dist_m": round(s.stdev(dists), 3) if len(dists) > 1 else 0,
    }


def heading_obs_acceptance(parsed) -> dict:
    """How NavPVT-derived yaw observations compare to the EKF's belief.

    /grunt1/rtk/heading_imu carries the gated NavPVT heading_of_motion
    that ekf_global consumes as imu1. Compare each one against the
    nearest ekf_global yaw to compute innovation magnitude — large
    innovations may be rejected by the Mahalanobis gate, while small
    innovations indicate the EKF is tracking the truth NavPVT reports.
    """
    hi = parsed.get("/grunt1/rtk/heading_imu", [])
    odom = parsed.get("/grunt1/odometry/global", [])
    if not hi or not odom:
        return {"name": "heading_obs_acceptance", "n": 0,
                "note": "missing heading_imu or odometry/global"}
    odom_t = [r[0] for r in odom]
    innovations = []
    head_accs = []
    for (t, yaw_rad, ha_deg) in hi:
        i = bisect.bisect_left(odom_t, t)
        if i >= len(odom) or abs(odom[i][0] - t) > 0.5:
            continue
        ekf_yaw = odom[i][3]
        innov = (yaw_rad - ekf_yaw + math.pi) % (2 * math.pi) - math.pi
        innovations.append(math.degrees(innov))
        head_accs.append(ha_deg)
    if not innovations:
        return {"name": "heading_obs_acceptance", "n": 0}
    abs_innov = [abs(x) for x in innovations]
    si = sorted(abs_innov)
    return {
        "name": "heading_obs_acceptance",
        "n": len(innovations),
        "median_innovation_deg": round(s.median(innovations), 2),
        "median_abs_innovation_deg": round(s.median(abs_innov), 2),
        "p90_abs_innovation_deg": round(si[int(len(si) * 0.9)], 2),
        "max_abs_innovation_deg": round(max(abs_innov), 2),
        "median_head_acc_deg": round(s.median(head_accs), 2),
    }


def collision_events(parsed) -> dict:
    """Distinct STATE transitions of collision_monitor — STOP / SLOWDOWN / etc."""
    cm = parsed.get("/grunt1/nav/collision_monitor_state", [])
    if not cm:
        return {"name": "collision_events", "n": 0}
    NAMES = {0: "CLEAR", 1: "STOP", 2: "SLOWDOWN", 3: "APPROACH", 4: "LIMIT"}
    transitions = []
    last = None
    t0 = cm[0][0]
    for (t, a) in cm:
        if a != last:
            transitions.append((round(t - t0, 1), NAMES.get(a, str(a))))
            last = a
    return {
        "name": "collision_events",
        "n": len(transitions),
        "transitions": transitions,
    }


# ────────────────────────────────────────────────────────────────────
# Composer
# ────────────────────────────────────────────────────────────────────

DEFAULT_METRICS = (
    datum_alignment,
    ekf_tracking,
    imu_health,
    rtk_continuity,
    cmd_vel_breakdown,
    control_oscillation,
    rotation_mode_fraction,
    carrot_tracking,
    heading_obs_acceptance,
    interventions,
    collision_events,
)


def run_all(parsed, anchor: GeoAnchor,
            waypoints_lla: Optional[list[tuple[float, float]]] = None) -> list[dict]:
    """Run every metric and return their results in order."""
    results = []
    for fn in DEFAULT_METRICS:
        # Some metrics need anchor, some don't — sniff arguments.
        try:
            if fn in (datum_alignment, ekf_tracking):
                results.append(fn(parsed, anchor))
            else:
                results.append(fn(parsed))
        except Exception as e:
            results.append({"name": fn.__name__, "error": str(e)})
    if waypoints_lla:
        try:
            results.append(crosstrack_error(parsed, waypoints_lla, anchor))
        except Exception as e:
            results.append({"name": "crosstrack_error", "error": str(e)})
    return results
