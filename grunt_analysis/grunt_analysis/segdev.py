"""Segment-deviation analysis.

For a given bag + mission YAML, computes how far the actual driven path
deviated from the ideal piecewise-linear path implied by the mission
waypoints, segment by segment. Also overlays the planner's published
plans (transformed from base_link → map frame) and the carrot vectors
(robot→lookahead_point at 1m progress intervals).

Outputs:
- report.md (per-segment table, hot-spot list, bow/curvature stats)
- segment_overlay.svg (top-down: segments + actual + plans, color-coded
  by plan internal bow)
- carrot_vectors.svg (top-down: segments + actual + robot→carrot lines
  every 1m of progress)
- deviation_strip.svg (signed offset vs distance along the ideal path)

Run via:
    python -m grunt_analysis.segdev <bag_dir> --mission-yaml <mission.yaml>
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
    quat_yaw, signed_offset_to_segment, closest_signed_offset,
    cumulative_segment_length, load_mission_segments,
    transform_plan_to_map, default_site_yaml,
)


# ──────────────────────────────────────────────────────────────────
# Bag streaming (full plan poses, not summarized)
# ──────────────────────────────────────────────────────────────────

def _stream_bag(bag_dir: Path, prefix: str = "/grunt1",
                plan_sample_dt_s: float = 5.0):
    """Read the bag once, return (odom, plan_samples, lookahead_pts).

    odom: [(t, x, y, yaw), ...]
    plan_samples: [(t, frame_id, [(px, py) base_link], ...] sampled at
                  ≥ plan_sample_dt_s spacing
    lookahead_pts: [(t, cx, cy)] in base_link
    """
    mcap_path = discover_mcap(bag_dir)
    odom, plan_samples, lookahead_pts = [], [], []
    last_plan_t = -1e9
    odom_topic = f"{prefix}/odometry/global"
    plan_topic = f"{prefix}/nav/received_global_plan"
    look_topic = f"{prefix}/nav/lookahead_point"
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
        elif topic == look_topic:
            lookahead_pts.append((t, msg.point.x, msg.point.y))
    return odom, plan_samples, lookahead_pts


def _carrot_vectors_at_progress(odom: list, lookahead_pts: list,
                                 progress_dl_m: float = 1.0) -> list:
    """Sample robot→carrot vectors every progress_dl_m of cumulative
    robot displacement. Returns [(t_rel, rx, ry, mx, my)] in map frame
    (mx, my = carrot transformed via robot yaw)."""
    if not odom or not lookahead_pts:
        return []
    t0 = odom[0][0]
    look_t = [lp[0] for lp in lookahead_pts]
    vectors = []
    cum_dist = 0.0
    last_xy = None
    next_threshold = 0.0
    for (rt, rx, ry, ryaw) in odom:
        if last_xy is not None:
            cum_dist += math.hypot(rx - last_xy[0], ry - last_xy[1])
        last_xy = (rx, ry)
        if cum_dist < next_threshold:
            continue
        i = bisect.bisect_left(look_t, rt)
        candidates = []
        if i < len(lookahead_pts):
            candidates.append(lookahead_pts[i])
        if i > 0:
            candidates.append(lookahead_pts[i - 1])
        if not candidates:
            next_threshold += progress_dl_m
            continue
        lp = min(candidates, key=lambda r: abs(r[0] - rt))
        if abs(lp[0] - rt) > 0.3:
            next_threshold += progress_dl_m
            continue
        cx, cy = lp[1], lp[2]
        if math.hypot(cx, cy) < 0.2:
            next_threshold += progress_dl_m
            continue
        cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
        mx = rx + cx * cos_y - cy * sin_y
        my = ry + cx * sin_y + cy * cos_y
        vectors.append((rt - t0, rx, ry, mx, my))
        next_threshold += progress_dl_m
    return vectors


# ──────────────────────────────────────────────────────────────────
# SVG rendering
# ──────────────────────────────────────────────────────────────────

def _topdown_canvas(xs: list[float], ys: list[float], pad: float = 4.0,
                     scale: float = 6.0) -> tuple:
    """Compute SVG canvas size + to_px function for a top-down view."""
    xmin, xmax = min(xs) - pad, max(xs) + pad
    ymin, ymax = min(ys) - pad, max(ys) + pad
    W = int((xmax - xmin) * scale) + 100  # +legend column
    H = int((ymax - ymin) * scale)
    def to_px(x, y, header_h=40):
        return ((x - xmin) * scale, H - (y - ymin) * scale + header_h)
    return W, H, to_px, (xmin, xmax, ymin, ymax)


def _draw_grid(elems, xmin, xmax, ymin, ymax, to_px, W, H, step=10):
    for gx in range(int(math.floor(xmin / step) * step),
                     int(math.ceil(xmax / step) * step) + 1, step):
        if gx < xmin or gx > xmax:
            continue
        x_, _ = to_px(gx, ymin)
        elems.append(f'<line x1="{x_}" y1="40" x2="{x_}" y2="{H+40}" '
                     f'stroke="#e6e6e6" stroke-width="0.5"/>')
        elems.append(f'<text x="{x_+2}" y="{H+38}" fill="#999" font-size="9">'
                     f'x={gx}</text>')
    for gy in range(int(math.floor(ymin / step) * step),
                     int(math.ceil(ymax / step) * step) + 1, step):
        if gy < ymin or gy > ymax:
            continue
        _, y_ = to_px(xmin, gy)
        elems.append(f'<line x1="0" y1="{y_}" x2="{W-100}" y2="{y_}" '
                     f'stroke="#e6e6e6" stroke-width="0.5"/>')
        elems.append(f'<text x="2" y="{y_-2}" fill="#999" font-size="9">'
                     f'y={gy}</text>')


def write_segment_overlay(path: Path, segs: list, wp_enu: list,
                           odom: list, plan_samples_map: list,
                           title: str):
    """Top-down: ideal segments (dashed green) + actual path (orange/red
    where |dev|>0.7m) + plans on top, colored by internal bow."""
    if not odom:
        return
    xs = [w[0] for w in wp_enu] + [d[1] for d in odom]
    ys = [w[1] for w in wp_enu] + [d[2] for d in odom]
    W, H, to_px, (xmin, xmax, ymin, ymax) = _topdown_canvas(xs, ys)
    elems = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H+40}" '
        f'viewBox="0 0 {W} {H+40}" font-family="ui-monospace,monospace" font-size="11">',
        f'<rect width="{W}" height="{H+40}" fill="#fafafa"/>',
        f'<text x="10" y="20" font-size="13" font-weight="bold">{title}</text>',
    ]
    _draw_grid(elems, xmin, xmax, ymin, ymax, to_px, W, H)

    # Segments (dashed gray-green)
    for ((ax_, ay_), (bx_, by_)) in segs:
        a = to_px(ax_, ay_); b = to_px(bx_, by_)
        elems.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" '
                     f'x2="{b[0]:.1f}" y2="{b[1]:.1f}" '
                     f'stroke="#5fa07a" stroke-width="1.5" opacity="0.7" '
                     f'stroke-dasharray="4,3"/>')

    # Actual path (thin orange/red so planner shows through on top)
    last = None
    for (t, x, y, _) in odom:
        signed, _, _ = closest_signed_offset(x, y, segs)
        px, py = to_px(x, y)
        if last is not None:
            color = "#d24a26" if abs(signed) > 0.7 else "#e08e1f"
            elems.append(f'<line x1="{last[0]:.1f}" y1="{last[1]:.1f}" '
                         f'x2="{px:.1f}" y2="{py:.1f}" '
                         f'stroke="{color}" stroke-width="0.7" opacity="0.6"/>')
        last = (px, py)

    # Plans on top, colored by internal bow
    for (t, poses) in plan_samples_map:
        if len(poses) < 2:
            continue
        ax_, ay_ = poses[0]
        bx_, by_ = poses[-1]
        bow = 0.0
        for (x, y) in poses[1:-1]:
            s = signed_offset_to_segment(x, y, ax_, ay_, bx_, by_)[0]
            if s is not None and abs(s) > bow:
                bow = abs(s)
        pts = " ".join(f"{to_px(p[0], p[1])[0]:.1f},{to_px(p[0], p[1])[1]:.1f}"
                       for p in poses)
        if bow > 0.5:
            elems.append(f'<polyline points="{pts}" fill="none" '
                         f'stroke="#d61680" stroke-width="2.2" opacity="0.95"/>')
        elif bow > 0.2:
            elems.append(f'<polyline points="{pts}" fill="none" '
                         f'stroke="#a040a0" stroke-width="1.4" opacity="0.9"/>')
        else:
            elems.append(f'<polyline points="{pts}" fill="none" '
                         f'stroke="#1f78b4" stroke-width="0.9" opacity="0.75"/>')

    # Waypoints
    for i, (x, y) in enumerate(wp_enu):
        px, py = to_px(x, y)
        elems.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="3.5" '
                     f'fill="#1a5e2c" stroke="white" stroke-width="1"/>')
        elems.append(f'<text x="{px+5:.1f}" y="{py+3:.1f}" fill="#1a5e2c" '
                     f'font-size="10" font-weight="bold">WP{i}</text>')

    # Legend
    lx, ly = W - 95, 50
    elems.append(f'<rect x="{lx}" y="{ly-10}" width="92" height="100" '
                 f'fill="white" stroke="#999" stroke-width="0.5"/>')
    elems.append(f'<text x="{lx+5}" y="{ly+5}" font-size="10" font-weight="bold">Legend</text>')
    legend_items = [
        ("#5fa07a", 1.6, True,  "ideal segs"),
        ("#e08e1f", 0.7, False, "actual path"),
        ("#d24a26", 0.7, False, "|dev| over 0.7m"),
        ("#1f78b4", 0.9, False, "plan bow to 0.2m"),
        ("#a040a0", 1.4, False, "plan bow 0.2-0.5"),
        ("#d61680", 2.2, False, "plan bow over 0.5m"),
    ]
    for i, (c, sw, dash, lbl) in enumerate(legend_items):
        y = ly + 18 + i * 13
        dash_attr = ' stroke-dasharray="4,3"' if dash else ''
        elems.append(f'<line x1="{lx+5}" y1="{y}" x2="{lx+25}" y2="{y}" '
                     f'stroke="{c}" stroke-width="{sw}"{dash_attr}/>')
        elems.append(f'<text x="{lx+30}" y="{y+3}" font-size="9">{lbl}</text>')

    elems.append('</svg>')
    Path(path).write_text("\n".join(elems))


def write_carrot_vectors(path: Path, segs: list, wp_enu: list,
                          odom: list, carrot_vectors: list, title: str):
    """Top-down: segments + actual path (thin) + robot→carrot vectors."""
    xs = [w[0] for w in wp_enu] + [d[1] for d in odom]
    ys = [w[1] for w in wp_enu] + [d[2] for d in odom]
    W, H, to_px, (xmin, xmax, ymin, ymax) = _topdown_canvas(xs, ys)
    elems = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H+40}" '
        f'viewBox="0 0 {W} {H+40}" font-family="ui-monospace,monospace" font-size="11">',
        f'<rect width="{W}" height="{H+40}" fill="#fafafa"/>',
        f'<text x="10" y="20" font-size="13" font-weight="bold">{title}</text>',
    ]
    _draw_grid(elems, xmin, xmax, ymin, ymax, to_px, W, H)
    for ((ax_, ay_), (bx_, by_)) in segs:
        a = to_px(ax_, ay_); b = to_px(bx_, by_)
        elems.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" '
                     f'x2="{b[0]:.1f}" y2="{b[1]:.1f}" '
                     f'stroke="#5fa07a" stroke-width="1.5" opacity="0.7" '
                     f'stroke-dasharray="4,3"/>')
    # Robot→carrot vectors
    for (_t, rx, ry, mx, my) in carrot_vectors:
        rp = to_px(rx, ry); cp = to_px(mx, my)
        elems.append(f'<line x1="{rp[0]:.1f}" y1="{rp[1]:.1f}" '
                     f'x2="{cp[0]:.1f}" y2="{cp[1]:.1f}" '
                     f'stroke="#c0359a" stroke-width="0.8" opacity="0.75"/>')
        elems.append(f'<circle cx="{cp[0]:.1f}" cy="{cp[1]:.1f}" r="1.4" '
                     f'fill="#c0359a" opacity="0.85"/>')
    # Actual path on top, thin
    last = None
    for (t, x, y, _) in odom:
        px, py = to_px(x, y)
        if last is not None:
            elems.append(f'<line x1="{last[0]:.1f}" y1="{last[1]:.1f}" '
                         f'x2="{px:.1f}" y2="{py:.1f}" '
                         f'stroke="#e08e1f" stroke-width="0.9" opacity="0.85"/>')
        last = (px, py)
    for i, (x, y) in enumerate(wp_enu):
        px, py = to_px(x, y)
        elems.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="3.5" '
                     f'fill="#1a5e2c" stroke="white" stroke-width="1"/>')
        elems.append(f'<text x="{px+5:.1f}" y="{py+3:.1f}" fill="#1a5e2c" '
                     f'font-size="10" font-weight="bold">WP{i}</text>')
    elems.append('</svg>')
    Path(path).write_text("\n".join(elems))


def write_strip_chart(path: Path, actual_dev: list, total_len: float,
                       cum: list, title: str):
    """Signed offset vs distance along ideal path."""
    W, H = 1400, 360
    ml, mr, mt, mb = 60, 30, 30, 50
    pw, ph = W - ml - mr, H - mt - mb
    y_max = 2.0
    def x_to_px(s): return ml + s / total_len * pw
    def y_to_px(o): return mt + ph/2 - o / y_max * (ph/2)
    elems = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        f'viewBox="0 0 {W} {H}" font-family="ui-monospace,monospace" font-size="11">',
        f'<rect width="{W}" height="{H}" fill="#fafafa"/>',
        f'<text x="{ml}" y="20" font-size="13" font-weight="bold">{title}</text>',
        f'<line x1="{ml}" y1="{mt}" x2="{ml}" y2="{mt+ph}" stroke="black"/>',
        f'<line x1="{ml}" y1="{y_to_px(0):.0f}" x2="{ml+pw}" y2="{y_to_px(0):.0f}" stroke="black"/>',
    ]
    for ytick in [-2, -1.5, -1, -0.5, 0.5, 1, 1.5, 2]:
        ypx = y_to_px(ytick)
        elems.append(f'<line x1="{ml}" y1="{ypx:.0f}" x2="{ml+pw}" y2="{ypx:.0f}" '
                     f'stroke="#e6e6e6" stroke-width="0.5"/>')
        elems.append(f'<text x="{ml-5}" y="{ypx+3:.0f}" text-anchor="end" '
                     f'font-size="9">{ytick:+.1f}</text>')
    for i, c in enumerate(cum):
        xpx = x_to_px(c)
        elems.append(f'<line x1="{xpx:.1f}" y1="{mt}" x2="{xpx:.1f}" y2="{mt+ph}" '
                     f'stroke="#bbb" stroke-width="0.5" stroke-dasharray="2,2"/>')
        elems.append(f'<text x="{xpx:.1f}" y="{mt-5}" text-anchor="middle" '
                     f'font-size="8" fill="#666">WP{i}</text>')
    last = None
    for (s_along, signed) in actual_dev:
        x = x_to_px(s_along); y = y_to_px(signed)
        is_red = abs(signed) > 0.7
        color = "#d24a26" if is_red else "#e08e1f"
        if last is not None:
            elems.append(f'<line x1="{last[0]:.1f}" y1="{last[1]:.1f}" '
                         f'x2="{x:.1f}" y2="{y:.1f}" stroke="{color}" '
                         f'stroke-width="0.7" opacity="0.7"/>')
        last = (x, y)
    elems.append(f'<text x="{ml+pw/2:.0f}" y="{H-15}" text-anchor="middle" '
                 f'font-size="11">distance along ideal path (m)</text>')
    elems.append(f'<text x="20" y="{mt+ph/2:.0f}" text-anchor="middle" '
                 f'font-size="11" transform="rotate(-90 20 {mt+ph/2:.0f})">'
                 f'signed offset (m, + = right)</text>')
    for ds in range(0, int(total_len) + 1, 25):
        elems.append(f'<text x="{x_to_px(ds):.0f}" y="{mt+ph+15}" '
                     f'text-anchor="middle" font-size="9">{ds}m</text>')
    elems.append('</svg>')
    Path(path).write_text("\n".join(elems))


# ──────────────────────────────────────────────────────────────────
# Report assembly
# ──────────────────────────────────────────────────────────────────

def _format_report(label: str, segs: list,
                    actual_signs_per_seg: dict[int, list],
                    plan_bow_stats: dict) -> list[str]:
    lines = [f"# Segment-deviation analysis — {label}", ""]
    lines += ["## Per-segment actual-path deviation", "",
              "| Seg | length | n | abs_med | abs_p90 | max | sign_med | bias |",
              "|-----|--------|---|---------|---------|-----|----------|------|"]
    hot = []
    for i, ((a, b)) in enumerate(segs):
        L = math.hypot(b[0]-a[0], b[1]-a[1])
        signs = actual_signs_per_seg.get(i, [])
        if not signs:
            continue
        abs_signs = [abs(s) for s in signs]
        sm = stx.median(signs)
        ams = stx.median(abs_signs)
        p90 = sorted(abs_signs)[9*len(abs_signs)//10]
        mx = max(abs_signs)
        n_pos = sum(1 for s in signs if s > 0.1)
        n_neg = sum(1 for s in signs if s < -0.1)
        tot = max(1, len(signs))
        bias = "RIGHT" if n_pos > 0.6*tot else ("LEFT" if n_neg > 0.6*tot else "mixed")
        lines.append(f"| {i} | {L:.1f}m | {len(signs)} | {ams:.2f} | {p90:.2f} | "
                     f"{mx:.2f} | {sm:+.2f} | {bias} |")
        if p90 >= 0.7:
            hot.append((i, L, p90, mx, sm, bias))
    if hot:
        lines += ["", "## Divergence hot spots (abs_p90 ≥ 0.7 m)", ""]
        for (i, L, p90, mx, sm, bias) in hot:
            lines.append(f"- **WP{i}→WP{i+1}** ({L:.1f}m): p90 {p90:.2f}m, "
                         f"max {mx:.2f}m, median {sm:+.2f}m, bias={bias}")
    lines += ["", "## Plan internal bow (planner curvature)", ""]
    if plan_bow_stats["n"] > 0:
        lines.append(f"- n: {plan_bow_stats['n']}")
        lines.append(f"- median: {plan_bow_stats['median']:.3f} m")
        lines.append(f"- p90: {plan_bow_stats['p90']:.3f} m")
        lines.append(f"- max: {plan_bow_stats['max']:.3f} m")
    else:
        lines.append("(no plan samples)")
    return lines


# ──────────────────────────────────────────────────────────────────
# CLI entry point
# ──────────────────────────────────────────────────────────────────

def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Per-segment deviation + planner overlay + carrot vectors")
    p.add_argument("bag_dir", type=Path)
    p.add_argument("--site-yaml", type=Path, default=None)
    p.add_argument("--mission-yaml", type=Path, required=True)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--prefix", default="/grunt1")
    p.add_argument("--plan-sample-dt", type=float, default=5.0,
                   help="seconds between plan samples (default 5)")
    p.add_argument("--carrot-progress-m", type=float, default=1.0,
                   help="meters of robot progress between carrot vectors")
    args = p.parse_args(argv)

    site_yaml = args.site_yaml or default_site_yaml(args.bag_dir)
    if site_yaml is None:
        print("error: no --site-yaml and no default", file=sys.stderr)
        return 2
    anchor = GeoAnchor.from_site_yaml(site_yaml)
    wp_enu, segs = load_mission_segments(args.mission_yaml, anchor)
    cum = cumulative_segment_length(segs)
    total_len = cum[-1]

    print(f"Streaming {args.bag_dir} ...", file=sys.stderr)
    odom, plan_raw, lookahead_pts = _stream_bag(
        args.bag_dir, prefix=args.prefix,
        plan_sample_dt_s=args.plan_sample_dt)
    if not odom:
        print("error: no odometry/global in bag", file=sys.stderr)
        return 1
    t0 = odom[0][0]
    odom_t = [r[0] for r in odom]

    # Transform plans base_link → map using closest odom pose
    plan_samples_map = []
    for (pt, frame, poses_bl) in plan_raw:
        poses_map = transform_plan_to_map(poses_bl, frame, pt, odom, odom_t)
        if poses_map:
            plan_samples_map.append((pt, poses_map))

    # Per-segment offsets along actual path + strip-chart points
    by_seg = {}
    strip_pts = []
    for (t, x, y, _) in odom:
        best_seg, best_signed, best_abs = -1, 0.0, float("inf")
        for i, ((ax_, ay_), (bx_, by_)) in enumerate(segs):
            signed, along = signed_offset_to_segment(x, y, ax_, ay_, bx_, by_)
            if signed is None:
                continue
            if abs(signed) < best_abs:
                best_seg, best_signed, best_abs = i, signed, abs(signed)
                best_along = along
        if best_seg < 0:
            continue
        by_seg.setdefault(best_seg, []).append(best_signed)
        strip_pts.append((cum[best_seg] + best_along, best_signed))

    # Plan-bow stats
    bows = []
    for (t, poses) in plan_samples_map:
        if len(poses) < 3:
            continue
        ax_, ay_ = poses[0]; bx_, by_ = poses[-1]
        bow = 0.0
        for (x, y) in poses[1:-1]:
            s = signed_offset_to_segment(x, y, ax_, ay_, bx_, by_)[0]
            if s is not None and abs(s) > bow:
                bow = abs(s)
        bows.append(bow)
    bows.sort()
    bow_stats = {"n": len(bows)}
    if bows:
        bow_stats["median"] = bows[len(bows)//2]
        bow_stats["p90"] = bows[9*len(bows)//10]
        bow_stats["max"] = max(bows)

    # Carrot vectors
    carrot_vecs = _carrot_vectors_at_progress(
        odom, lookahead_pts, progress_dl_m=args.carrot_progress_m)

    # Write outputs
    out_dir = args.out or args.bag_dir
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    label = args.bag_dir.name

    report = _format_report(label, segs, by_seg, bow_stats)
    (out_dir / "segdev_report.md").write_text("\n".join(report))
    print(f"wrote {out_dir / 'segdev_report.md'}", file=sys.stderr)

    write_segment_overlay(
        out_dir / "segment_overlay.svg", segs, wp_enu, odom,
        plan_samples_map,
        title=f"Segment deviation + planner overlay — {label}")
    print(f"wrote {out_dir / 'segment_overlay.svg'}", file=sys.stderr)

    write_carrot_vectors(
        out_dir / "carrot_vectors.svg", segs, wp_enu, odom, carrot_vecs,
        title=f"Robot→carrot vectors @ {args.carrot_progress_m}m progress — {label}")
    print(f"wrote {out_dir / 'carrot_vectors.svg'}", file=sys.stderr)

    write_strip_chart(
        out_dir / "deviation_strip.svg", strip_pts, total_len, cum,
        title=f"Signed deviation vs path arc-distance — {label}")
    print(f"wrote {out_dir / 'deviation_strip.svg'}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
