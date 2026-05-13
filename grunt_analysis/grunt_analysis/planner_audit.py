"""Planner audit: plan-vs-ideal-segment deviation + EKF jump detection.

Different from segdev's plan-bow metric: this measures how far the
PLAN ITSELF strays from the straight WP_n → WP_{n+1} segment line —
which is what the operator-biased mission YAML was meant to be.

A plan can be internally straight (bow=0) but still 1m off the segment
line if the planner is routing around a costmap mark. That's planner
contribution to deviation.

Also detects sudden jumps in /odometry/global that would indicate the
EKF moved the robot's frame between plan publishes — a worthwhile sanity
check when investigating why a plan is shaped strangely.

Outputs:
- planner_audit.md — top deviating plans, sign distribution, EKF jumps
- planner_only.svg — segments + plans (colored by deviation), no actual-
  path occlusion, with thin gray actual-path for context

Run via:
    python -m grunt_analysis.planner_audit <bag_dir> --mission-yaml <m.yaml>
"""
from __future__ import annotations

import argparse
import bisect
import math
import statistics as stx
import sys
from pathlib import Path

from mcap_ros2.reader import read_ros2_messages

from .bag import discover_mcap
from .geo import GeoAnchor
from ._deep_common import (
    quat_yaw, signed_perp_to_seg, signed_offset_to_segment,
    closest_signed_offset, find_traversed_segment,
    load_mission_segments, transform_plan_to_map, default_site_yaml,
)


def _stream_bag(bag_dir: Path, prefix: str = "/grunt1",
                plan_sample_dt_s: float = 2.0):
    """Return (odom, plan_samples [(t, frame, poses_bl)], coll_events)."""
    mcap_path = discover_mcap(bag_dir)
    odom, plan_samples, coll = [], [], []
    last_plan_t = -1e9
    odom_topic = f"{prefix}/odometry/global"
    plan_topic = f"{prefix}/nav/received_global_plan"
    coll_topic = f"{prefix}/nav/collision_monitor_state"
    for m in read_ros2_messages(str(mcap_path)):
        topic = m.channel.topic
        msg = m.ros_msg
        t = m.log_time_ns / 1e9
        if topic == odom_topic:
            p = msg.pose.pose
            odom.append((t, p.position.x, p.position.y, quat_yaw(p.orientation)))
        elif topic == plan_topic:
            if t - last_plan_t < plan_sample_dt_s:
                continue
            last_plan_t = t
            frame = msg.header.frame_id
            poses = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
            if poses:
                plan_samples.append((t, frame, poses))
        elif topic == coll_topic:
            coll.append((t, int(msg.action_type)))
    return odom, plan_samples, coll


# ──────────────────────────────────────────────────────────────────
# EKF jump check
# ──────────────────────────────────────────────────────────────────

def detect_ekf_jumps(odom: list, pos_thresh_m: float = 0.3,
                      yaw_thresh_deg: float = 20.0,
                      dt_thresh_s: float = 0.2) -> tuple[list, list]:
    """Scan consecutive odom samples for impossible chassis motion.

    Returns (pos_jumps, yaw_jumps), each [(t_rel, magnitude, dt_ms)].
    The robot can't translate >0.3m in <0.2s (would be >1.5 m/s — our
    cap) or rotate >20° in <0.2s realistically.
    """
    if not odom:
        return [], []
    t0 = odom[0][0]
    pos, yaws = [], []
    yaw_rad = math.radians(yaw_thresh_deg)
    for i in range(1, len(odom)):
        dt = odom[i][0] - odom[i-1][0]
        if dt > dt_thresh_s or dt <= 0:
            continue
        d = math.hypot(odom[i][1]-odom[i-1][1], odom[i][2]-odom[i-1][2])
        if d > pos_thresh_m:
            pos.append((odom[i][0] - t0, d, dt))
        dy = odom[i][3] - odom[i-1][3]
        while dy > math.pi: dy -= 2*math.pi
        while dy < -math.pi: dy += 2*math.pi
        if abs(dy) > yaw_rad:
            yaws.append((odom[i][0] - t0, math.degrees(dy), dt))
    return pos, yaws


# ──────────────────────────────────────────────────────────────────
# SVG: planner-only view
# ──────────────────────────────────────────────────────────────────

def write_planner_only_svg(path: Path, segs: list, wp_enu: list,
                            odom: list, plan_rows: list, title: str):
    """Top-down: dashed segments + thin gray actual + plans colored by
    deviation from ideal segment line (no actual-path occlusion).

    plan_rows: [{t_rel, max_dev, poses_map}, ...]
    """
    xs = [w[0] for w in wp_enu] + [d[1] for d in odom]
    ys = [w[1] for w in wp_enu] + [d[2] for d in odom]
    pad = 4.0
    xmin, xmax = min(xs) - pad, max(xs) + pad
    ymin, ymax = min(ys) - pad, max(ys) + pad
    scale = 6.0
    W = int((xmax - xmin) * scale) + 110
    H = int((ymax - ymin) * scale)
    def to_px(x, y): return ((x - xmin) * scale, H - (y - ymin) * scale + 50)
    elems = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H+50}" '
        f'viewBox="0 0 {W} {H+50}" font-family="ui-monospace,monospace" font-size="11">',
        f'<rect width="{W}" height="{H+50}" fill="#fafafa"/>',
        f'<text x="10" y="20" font-size="13" font-weight="bold">{title}</text>',
        f'<text x="10" y="36" font-size="10" fill="#666">'
        f'Plans colored by max deviation from ideal WP→WP segment.</text>',
    ]
    # Gridlines
    step = 10
    for gx in range(int(math.floor(xmin/step)*step), int(math.ceil(xmax/step)*step)+1, step):
        if gx < xmin or gx > xmax: continue
        x_, _ = to_px(gx, ymin)
        elems.append(f'<line x1="{x_}" y1="50" x2="{x_}" y2="{H+50}" '
                     f'stroke="#e6e6e6" stroke-width="0.5"/>')
        elems.append(f'<text x="{x_+2}" y="{H+48}" fill="#999" font-size="9">x={gx}</text>')
    for gy in range(int(math.floor(ymin/step)*step), int(math.ceil(ymax/step)*step)+1, step):
        if gy < ymin or gy > ymax: continue
        _, y_ = to_px(xmin, gy)
        elems.append(f'<line x1="0" y1="{y_}" x2="{W-110}" y2="{y_}" '
                     f'stroke="#e6e6e6" stroke-width="0.5"/>')
        elems.append(f'<text x="2" y="{y_-2}" fill="#999" font-size="9">y={gy}</text>')
    # Segments
    for ((ax_, ay_), (bx_, by_)) in segs:
        a = to_px(ax_, ay_); b = to_px(bx_, by_)
        elems.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" '
                     f'x2="{b[0]:.1f}" y2="{b[1]:.1f}" '
                     f'stroke="#5fa07a" stroke-width="1.6" opacity="0.85" '
                     f'stroke-dasharray="5,3"/>')
    # Actual path (thin gray)
    last = None
    for r in odom[::5]:
        px, py = to_px(r[1], r[2])
        if last is not None:
            elems.append(f'<line x1="{last[0]:.1f}" y1="{last[1]:.1f}" '
                         f'x2="{px:.1f}" y2="{py:.1f}" '
                         f'stroke="#c8c8c8" stroke-width="0.8" opacity="0.7"/>')
        last = (px, py)
    # Plans colored by deviation from ideal segment
    for row in plan_rows:
        poses = row["poses_map"]
        if len(poses) < 2:
            continue
        ad = abs(row["max_dev"])
        if ad < 0.2:
            color, sw, op = "#1f78b4", 0.8, 0.65
        elif ad < 0.5:
            color, sw, op = "#a040a0", 1.4, 0.85
        else:
            color, sw, op = "#d61680", 2.2, 0.95
        pts = " ".join(f"{to_px(p[0], p[1])[0]:.1f},{to_px(p[0], p[1])[1]:.1f}"
                       for p in poses)
        elems.append(f'<polyline points="{pts}" fill="none" stroke="{color}" '
                     f'stroke-width="{sw}" opacity="{op}"/>')
        if ad > 0.5:
            mid = poses[len(poses)//2]
            mpx, mpy = to_px(mid[0], mid[1])
            elems.append(f'<text x="{mpx+5:.0f}" y="{mpy:.0f}" fill="#d61680" '
                         f'font-size="9" font-weight="bold">'
                         f'dev={row["max_dev"]:+.2f}m@t={row["t_rel"]:.0f}s</text>')
    # Waypoints
    for i, (x, y) in enumerate(wp_enu):
        px, py = to_px(x, y)
        elems.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="3.5" '
                     f'fill="#1a5e2c" stroke="white" stroke-width="1"/>')
        elems.append(f'<text x="{px+5:.1f}" y="{py+3:.1f}" fill="#1a5e2c" '
                     f'font-size="10" font-weight="bold">WP{i}</text>')
    # Legend
    lx, ly = W - 105, 60
    elems.append(f'<rect x="{lx}" y="{ly-10}" width="100" height="100" '
                 f'fill="white" stroke="#999" stroke-width="0.5"/>')
    elems.append(f'<text x="{lx+5}" y="{ly+5}" font-size="10" font-weight="bold">Legend</text>')
    items = [
        ("#5fa07a", 1.6, True,  "ideal segs"),
        ("#c8c8c8", 0.8, False, "actual (ctx)"),
        ("#1f78b4", 0.8, False, "plan dev to 0.2m"),
        ("#a040a0", 1.4, False, "plan 0.2-0.5m"),
        ("#d61680", 2.2, False, "plan over 0.5m"),
    ]
    for i, (c, sw, dash, lbl) in enumerate(items):
        y = ly + 18 + i*16
        dash_attr = ' stroke-dasharray="4,3"' if dash else ''
        elems.append(f'<line x1="{lx+5}" y1="{y}" x2="{lx+25}" y2="{y}" '
                     f'stroke="{c}" stroke-width="{sw}"{dash_attr}/>')
        elems.append(f'<text x="{lx+30}" y="{y+3}" font-size="9">{lbl}</text>')
    elems.append('</svg>')
    Path(path).write_text("\n".join(elems))


# ──────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────

def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Plan-vs-ideal-segment deviation + EKF jump check")
    p.add_argument("bag_dir", type=Path)
    p.add_argument("--site-yaml", type=Path, default=None)
    p.add_argument("--mission-yaml", type=Path, required=True)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--prefix", default="/grunt1")
    p.add_argument("--plan-sample-dt", type=float, default=2.0)
    args = p.parse_args(argv)

    site_yaml = args.site_yaml or default_site_yaml(args.bag_dir)
    if site_yaml is None:
        print("error: no --site-yaml and no default", file=sys.stderr)
        return 2
    anchor = GeoAnchor.from_site_yaml(site_yaml)
    wp_enu, segs = load_mission_segments(args.mission_yaml, anchor)

    print(f"Streaming {args.bag_dir} ...", file=sys.stderr)
    odom, plan_raw, coll = _stream_bag(args.bag_dir,
                                        prefix=args.prefix,
                                        plan_sample_dt_s=args.plan_sample_dt)
    if not odom:
        print("error: no odometry/global in bag", file=sys.stderr)
        return 1
    t0 = odom[0][0]
    odom_t = [r[0] for r in odom]

    # Transform plans
    plan_map = []
    for (pt, frame, poses_bl) in plan_raw:
        poses_map = transform_plan_to_map(poses_bl, frame, pt, odom, odom_t)
        if poses_map:
            plan_map.append((pt, poses_map))

    # Plan-vs-segment deviation
    rows = []
    for (pt, poses) in plan_map:
        if len(poses) < 3:
            continue
        seg_idx = find_traversed_segment(poses, segs)
        if seg_idx < 0:
            continue
        (ax_, ay_), (bx_, by_) = segs[seg_idx]
        seg_devs = []
        for (x, y) in poses:
            d = signed_perp_to_seg(x, y, ax_, ay_, bx_, by_)
            if d is not None:
                seg_devs.append(d)
        if not seg_devs:
            continue
        max_dev = max(seg_devs, key=abs)
        med_dev = stx.median(seg_devs)
        rows.append({
            "t_rel": pt - t0, "seg": seg_idx, "n": len(poses),
            "max_dev": max_dev, "med_dev": med_dev,
            "poses_map": poses,
        })

    pos_jumps, yaw_jumps = detect_ekf_jumps(odom)

    # Report
    out_dir = args.out or args.bag_dir
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    label = args.bag_dir.name
    lines = [f"# Planner audit — {label}", ""]
    lines += [
        "## Plan-vs-ideal-segment deviation",
        "",
        "| t_rel | seg | n | max_dev | med_dev |",
        "|-------|-----|---|---------|---------|",
    ]
    for r in sorted(rows, key=lambda r: -abs(r["max_dev"]))[:25]:
        lines.append(f"| {r['t_rel']:5.1f} | wp{r['seg']:>2}-{r['seg']+1:<2} | "
                     f"{r['n']:3d} | {r['max_dev']:+.3f}m | {r['med_dev']:+.3f}m |")
    devs = sorted(abs(r["max_dev"]) for r in rows)
    if devs:
        lines += ["", "**Summary**",
                  f"- n: {len(devs)}",
                  f"- median: {devs[len(devs)//2]:.3f} m",
                  f"- p90: {devs[9*len(devs)//10]:.3f} m",
                  f"- max: {max(devs):.3f} m"]
        pos = sum(1 for r in rows if r["med_dev"] > 0.1)
        neg = sum(1 for r in rows if r["med_dev"] < -0.1)
        lines += [f"- right-of-segment median: {pos}",
                  f"- left-of-segment median: {neg}",
                  f"- centered (|dev|<0.1m): {len(rows)-pos-neg}"]
    lines += ["", "## EKF jumps", ""]
    lines.append(f"Position jumps (>0.3m / <0.2s): {len(pos_jumps)}")
    if pos_jumps:
        lines += ["| t_rel | jump | dt |", "|-------|------|------|"]
        for (t, d, dt) in pos_jumps[:20]:
            lines.append(f"| {t:5.1f} | {d:.3f}m | {dt*1000:.0f}ms |")
    lines.append("")
    lines.append(f"Yaw jumps (>20° / <0.2s): {len(yaw_jumps)}")
    if yaw_jumps:
        lines += ["| t_rel | dyaw | dt |", "|-------|------|------|"]
        for (t, d, dt) in yaw_jumps[:20]:
            lines.append(f"| {t:5.1f} | {d:+.1f}° | {dt*1000:.0f}ms |")

    (out_dir / "planner_audit.md").write_text("\n".join(lines))
    print(f"wrote {out_dir / 'planner_audit.md'}", file=sys.stderr)

    write_planner_only_svg(
        out_dir / "planner_only.svg", segs, wp_enu, odom, rows,
        title=f"Planner-only view — {label}")
    print(f"wrote {out_dir / 'planner_only.svg'}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
