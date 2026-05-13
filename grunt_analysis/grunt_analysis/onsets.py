"""Fresh-onset analysis for collision events + sonar mark overlay.

Distinguishes FRESH onsets (no collision activity for 10 s prior) from
cluster events. Without that filter, a single foliage encounter becomes
10+ events as the robot extricates — any "% of plans correlate with
collision activity" claim is inflated by cluster recounting.

Also extracts sonar marks from /sonar/cloud (PointCloud2), transforms
to map frame, and overlays them on a top-down view alongside planner
output and fresh-onset markers. The dot density along the lane edges
is what the planner is reacting to.

Outputs:
- onset_audit.md — fresh-vs-cluster table, robot offset at each fresh
  onset, plan deviation in the prior 10 s, mark density by offset bin
- planner_with_marks.svg — segments + actual + plans + sonar marks +
  fresh-onset X markers

Run via:
    python -m grunt_analysis.onsets <bag_dir> --mission-yaml <m.yaml>
"""
from __future__ import annotations

import argparse
import bisect
import math
import sys
from pathlib import Path

from mcap_ros2.reader import read_ros2_messages

from .bag import discover_mcap
from .geo import GeoAnchor
from ._deep_common import (
    quat_yaw, signed_perp_to_seg, signed_offset_to_segment,
    closest_signed_offset, find_traversed_segment, load_mission_segments,
    transform_plan_to_map, parse_pointcloud2, fresh_onsets,
    default_site_yaml,
)


def _stream_bag(bag_dir: Path, prefix: str = "/grunt1",
                plan_sample_dt_s: float = 2.0):
    """Return (odom, plan_samples, coll, sonar_clouds)."""
    mcap_path = discover_mcap(bag_dir)
    odom, plan_samples, coll, sonar = [], [], [], []
    last_plan_t = -1e9
    odom_topic = f"{prefix}/odometry/global"
    plan_topic = f"{prefix}/nav/received_global_plan"
    coll_topic = f"{prefix}/nav/collision_monitor_state"
    sonar_topic = f"{prefix}/sonar/cloud"
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
            poses = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
            if poses:
                plan_samples.append((t, msg.header.frame_id, poses))
        elif topic == coll_topic:
            coll.append((t, int(msg.action_type)))
        elif topic == sonar_topic:
            pts = parse_pointcloud2(msg)
            if pts:
                sonar.append((t, pts))
    return odom, plan_samples, coll, sonar


def _marks_to_map(sonar_clouds: list, odom: list,
                   odom_t: list[float],
                   max_skew_s: float = 0.3) -> list[tuple[float, float, float, float]]:
    """Transform sonar PointCloud2 contents from base_link to map frame
    using closest odom pose. Returns [(t_rel, mx, my, range_bl)]."""
    if not odom or not sonar_clouds:
        return []
    t0 = odom[0][0]
    out = []
    for (t, pts) in sonar_clouds:
        i = bisect.bisect_left(odom_t, t)
        candidates = []
        if i < len(odom): candidates.append(odom[i])
        if i > 0: candidates.append(odom[i-1])
        if not candidates:
            continue
        ref = min(candidates, key=lambda r: abs(r[0] - t))
        if abs(ref[0] - t) > max_skew_s:
            continue
        rx, ry, ryaw = ref[1], ref[2], ref[3]
        cy_, sy_ = math.cos(ryaw), math.sin(ryaw)
        for (px, py, _pz) in pts:
            mx = rx + px * cy_ - py * sy_
            my = ry + px * sy_ + py * cy_
            out.append((t - t0, mx, my, math.hypot(px, py)))
    return out


# ──────────────────────────────────────────────────────────────────
# SVG
# ──────────────────────────────────────────────────────────────────

def write_planner_with_marks_svg(path: Path, segs: list, wp_enu: list,
                                   odom: list, plan_rows: list,
                                   map_marks: list, fresh: list,
                                   t0: float, title: str):
    """Planner-only + sonar marks (small black dots) + fresh-onset X."""
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
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H+60}" '
        f'viewBox="0 0 {W} {H+60}" font-family="ui-monospace,monospace" font-size="11">',
        f'<rect width="{W}" height="{H+60}" fill="#fafafa"/>',
        f'<text x="10" y="20" font-size="13" font-weight="bold">{title}</text>',
        f'<text x="10" y="36" font-size="10" fill="#666">'
        f'Sonar marks (black dots) show the obstacle field the planner reacts to. '
        f'Green X = fresh-onset collision (no prior collision in 10s).</text>',
    ]
    # Segments
    for ((ax_, ay_), (bx_, by_)) in segs:
        a = to_px(ax_, ay_); b = to_px(bx_, by_)
        elems.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" '
                     f'x2="{b[0]:.1f}" y2="{b[1]:.1f}" '
                     f'stroke="#5fa07a" stroke-width="1.6" opacity="0.85" '
                     f'stroke-dasharray="5,3"/>')
    # Sonar marks (every 2nd to thin)
    for (_t, mx, my, _r) in map_marks[::2]:
        px, py = to_px(mx, my)
        elems.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="0.9" '
                     f'fill="#222" opacity="0.4"/>')
    # Actual path (thin orange context)
    last = None
    for r in odom[::5]:
        px, py = to_px(r[1], r[2])
        if last is not None:
            elems.append(f'<line x1="{last[0]:.1f}" y1="{last[1]:.1f}" '
                         f'x2="{px:.1f}" y2="{py:.1f}" '
                         f'stroke="#e08e1f" stroke-width="0.7" opacity="0.7"/>')
        last = (px, py)
    # Plans
    for row in plan_rows:
        poses = row["poses_map"]
        if len(poses) < 2:
            continue
        ad = abs(row["max_dev"])
        if ad < 0.2:
            color, sw, op = "#1f78b4", 0.8, 0.6
        elif ad < 0.5:
            color, sw, op = "#a040a0", 1.4, 0.85
        else:
            color, sw, op = "#d61680", 2.2, 0.95
        pts = " ".join(f"{to_px(p[0], p[1])[0]:.1f},{to_px(p[0], p[1])[1]:.1f}"
                       for p in poses)
        elems.append(f'<polyline points="{pts}" fill="none" stroke="{color}" '
                     f'stroke-width="{sw}" opacity="{op}"/>')
    # Fresh-onset X markers
    odom_t = [r[0] for r in odom]
    for (t, s) in fresh:
        i = bisect.bisect_left(odom_t, t)
        if i >= len(odom):
            continue
        r = odom[i]
        px, py = to_px(r[1], r[2])
        elems.append(f'<g stroke="#0a8a3e" stroke-width="2" fill="none">'
                     f'<line x1="{px-5}" y1="{py-5}" x2="{px+5}" y2="{py+5}"/>'
                     f'<line x1="{px-5}" y1="{py+5}" x2="{px+5}" y2="{py-5}"/>'
                     f'</g>')
        elems.append(f'<text x="{px+8:.0f}" y="{py+3:.0f}" fill="#0a8a3e" '
                     f'font-size="9" font-weight="bold">@{t-t0:.0f}s</text>')
    # Waypoints
    for i, (x, y) in enumerate(wp_enu):
        px, py = to_px(x, y)
        elems.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="3.5" '
                     f'fill="#1a5e2c" stroke="white" stroke-width="1"/>')
        elems.append(f'<text x="{px+5:.1f}" y="{py+3:.1f}" fill="#1a5e2c" '
                     f'font-size="10" font-weight="bold">WP{i}</text>')
    # Legend
    lx, ly = W - 105, 70
    elems.append(f'<rect x="{lx}" y="{ly-12}" width="100" height="140" '
                 f'fill="white" stroke="#999" stroke-width="0.5"/>')
    elems.append(f'<text x="{lx+5}" y="{ly+3}" font-size="10" font-weight="bold">Legend</text>')
    items = [
        ("#5fa07a", 1.6, True, None, "ideal segs"),
        ("#e08e1f", 0.7, False, None, "actual path"),
        ("#1f78b4", 0.8, False, None, "plan dev to 0.2m"),
        ("#a040a0", 1.4, False, None, "plan 0.2-0.5m"),
        ("#d61680", 2.2, False, None, "plan over 0.5m"),
        (None, 0, False, "dot", "sonar mark"),
        (None, 0, False, "x", "fresh stop"),
    ]
    for i, (c, sw, dash, marker, lbl) in enumerate(items):
        y = ly + 18 + i*16
        if marker == "dot":
            elems.append(f'<circle cx="{lx+15}" cy="{y}" r="1.5" fill="#222" opacity="0.6"/>')
        elif marker == "x":
            elems.append(f'<g stroke="#0a8a3e" stroke-width="2">'
                         f'<line x1="{lx+10}" y1="{y-4}" x2="{lx+20}" y2="{y+4}"/>'
                         f'<line x1="{lx+10}" y1="{y+4}" x2="{lx+20}" y2="{y-4}"/>'
                         f'</g>')
        else:
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
        description="Fresh-onset analysis + sonar mark overlay")
    p.add_argument("bag_dir", type=Path)
    p.add_argument("--site-yaml", type=Path, default=None)
    p.add_argument("--mission-yaml", type=Path, required=True)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--prefix", default="/grunt1")
    p.add_argument("--quiet-window-s", type=float, default=10.0,
                   help="seconds of clear state before a non-zero event "
                        "counts as a FRESH onset (default 10)")
    args = p.parse_args(argv)

    site_yaml = args.site_yaml or default_site_yaml(args.bag_dir)
    if site_yaml is None:
        print("error: no --site-yaml and no default", file=sys.stderr)
        return 2
    anchor = GeoAnchor.from_site_yaml(site_yaml)
    wp_enu, segs = load_mission_segments(args.mission_yaml, anchor)

    print(f"Streaming {args.bag_dir} ...", file=sys.stderr)
    odom, plan_raw, coll, sonar = _stream_bag(args.bag_dir, prefix=args.prefix)
    if not odom:
        print("error: no odometry/global in bag", file=sys.stderr)
        return 1
    t0 = odom[0][0]
    odom_t = [r[0] for r in odom]

    # Transform plans + compute max dev from ideal segment
    plan_rows = []
    for (pt, frame, poses_bl) in plan_raw:
        poses_map = transform_plan_to_map(poses_bl, frame, pt, odom, odom_t)
        if not poses_map or len(poses_map) < 3:
            continue
        seg_idx = find_traversed_segment(poses_map, segs)
        if seg_idx < 0:
            continue
        (ax_, ay_), (bx_, by_) = segs[seg_idx]
        max_dev = 0.0
        for (x, y) in poses_map:
            d = signed_perp_to_seg(x, y, ax_, ay_, bx_, by_)
            if d is not None and abs(d) > abs(max_dev):
                max_dev = d
        plan_rows.append({"t_rel": pt - t0, "max_dev": max_dev,
                          "poses_map": poses_map, "t_abs": pt})

    # Fresh onsets
    nz = [(t, s) for (t, s) in coll if s != 0]
    fresh = fresh_onsets(coll, quiet_window_s=args.quiet_window_s)

    # Sonar marks to map frame
    map_marks = _marks_to_map(sonar, odom, odom_t)

    # Per-fresh-onset robot context
    onset_rows = []
    plan_t_arr = [r["t_abs"] for r in plan_rows]
    for (t, s) in fresh:
        i = bisect.bisect_left(odom_t, t)
        if i >= len(odom):
            continue
        r = odom[i]
        signed_now, abs_now, seg = closest_signed_offset(r[1], r[2], segs)
        j = bisect.bisect_left(odom_t, t - 5.0)
        signed_5 = 0.0
        if j < len(odom):
            r5 = odom[j]
            signed_5, _, _ = closest_signed_offset(r5[1], r5[2], segs)
        # Most-recent plan before onset
        pi = bisect.bisect_left(plan_t_arr, t)
        last_plan_dev = None
        if pi > 0:
            prior = plan_rows[pi - 1]
            if t - prior["t_abs"] <= 10.0:
                last_plan_dev = prior["max_dev"]
        onset_rows.append({"t_rel": t - t0, "state": s, "seg": seg,
                           "offset": signed_now,
                           "offset_5s": signed_5,
                           "last_plan_dev": last_plan_dev})

    # Mark density by offset
    bins = [(0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 3.0)]
    mark_bin_counts = {b: 0 for b in bins}
    coll_bin_counts = {b: 0 for b in bins}
    fresh_bin_counts = {b: 0 for b in bins}
    for (tr, mx, my, _rng) in map_marks[::5]:
        i = bisect.bisect_left(odom_t, tr + t0)
        if i >= len(odom):
            continue
        r = odom[i]
        _, abs_off, _ = closest_signed_offset(r[1], r[2], segs)
        for b in bins:
            if b[0] <= abs_off < b[1]:
                mark_bin_counts[b] += 1
                break
    fresh_set = {(t, s) for t, s in fresh}
    for (t, s) in nz:
        i = bisect.bisect_left(odom_t, t)
        if i >= len(odom):
            continue
        r = odom[i]
        _, abs_off, _ = closest_signed_offset(r[1], r[2], segs)
        for b in bins:
            if b[0] <= abs_off < b[1]:
                coll_bin_counts[b] += 1
                if (t, s) in fresh_set:
                    fresh_bin_counts[b] += 1
                break

    # Write report
    out_dir = args.out or args.bag_dir
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    label = args.bag_dir.name
    lines = [f"# Onset analysis — {label}", ""]
    lines += [f"- non-zero collision events: {len(nz)}",
              f"- fresh onsets (>= {args.quiet_window_s}s of clear state prior): {len(fresh)}",
              f"- sonar marks (map frame): {len(map_marks)}",
              ""]
    lines += ["## Fresh onsets",
              "",
              "| t_rel | state | seg | offset_now | offset_5s_prior | last_plan_dev |",
              "|-------|-------|-----|------------|------------------|----------------|"]
    for r in onset_rows:
        ldp = f"{r['last_plan_dev']:+.2f}m" if r["last_plan_dev"] is not None else "—"
        lines.append(f"| {r['t_rel']:5.1f} | {r['state']:5d} | "
                     f"wp{r['seg']:>2} | {r['offset']:+.2f}m | "
                     f"{r['offset_5s']:+.2f}m | {ldp} |")
    lines += ["", "## Collision events by robot offset",
              "",
              "| offset bin | all events | fresh onsets |",
              "|------------|------------|--------------|"]
    for b in bins:
        lines.append(f"| {b[0]:.1f}-{b[1]:.1f}m | {coll_bin_counts[b]} | "
                     f"{fresh_bin_counts[b]} |")
    lines += ["", "## Sonar mark density by robot offset (every-5th sample)",
              "",
              "| offset bin | marks |",
              "|------------|-------|"]
    for b in bins:
        lines.append(f"| {b[0]:.1f}-{b[1]:.1f}m | {mark_bin_counts[b]} |")
    (out_dir / "onset_audit.md").write_text("\n".join(lines))
    print(f"wrote {out_dir / 'onset_audit.md'}", file=sys.stderr)

    write_planner_with_marks_svg(
        out_dir / "planner_with_marks.svg",
        segs, wp_enu, odom, plan_rows, map_marks, fresh, t0,
        title=f"Plans + sonar marks + fresh onsets — {label}")
    print(f"wrote {out_dir / 'planner_with_marks.svg'}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
