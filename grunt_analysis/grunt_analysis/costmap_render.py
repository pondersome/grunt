"""Render the ACTUAL recorded global_costmap OccupancyGrid as PNGs at
moments of interest.

`diagnostic_bagger` records `/grunt1/nav/global_costmap/costmap` at 1 Hz
and `/grunt1/nav/local_costmap/costmap` at 2 Hz. This script pulls the
nearest costmap at each moment of interest and renders it with the
mission overlay (segments, waypoints, actual path, active plan, robot
pose).

Moments of interest:
- Every fresh-onset collision event (see `onsets.py` for the definition)
- Additional times passed via --at

This is the most-accurate obstacle-field view — direct from what the
planner saw. For sonar-only reconstruction (when costmap topics weren't
recorded) see the costmap_reconstruction script in /tmp (not promoted).

Outputs to <out>/:
- actual_costmap_t<NNN>s_<label>.png — one per moment

Run via:
    python -m grunt_analysis.costmap_render <bag_dir> --mission-yaml <m.yaml>
    python -m grunt_analysis.costmap_render <bag_dir> --at 120 --at 250 ...
"""
from __future__ import annotations

import argparse
import bisect
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from mcap_ros2.reader import read_ros2_messages

from .bag import discover_mcap
from .geo import GeoAnchor
from ._deep_common import (
    quat_yaw, load_mission_segments, transform_plan_to_map,
    fresh_onsets, default_site_yaml,
)


COSTMAP_CMAP = LinearSegmentedColormap.from_list(
    "costmap_grid",
    [(0.95, 0.95, 0.95),
     (0.85, 0.92, 0.97),
     (1.0, 0.87, 0.5),
     (0.95, 0.4, 0.3),
     (0.5, 0.05, 0.1)],
    N=256,
)


def _stream_bag(bag_dir: Path, prefix: str = "/grunt1"):
    """Return (odom, costmap_msgs, coll, plan_samples)."""
    mcap_path = discover_mcap(bag_dir)
    odom, costmaps, coll, plans = [], [], [], []
    odom_topic = f"{prefix}/odometry/global"
    cm_topic = f"{prefix}/nav/global_costmap/costmap"
    coll_topic = f"{prefix}/nav/collision_monitor_state"
    plan_topic = f"{prefix}/nav/received_global_plan"
    last_plan_t = -1e9
    for m in read_ros2_messages(str(mcap_path)):
        topic = m.channel.topic
        msg = m.ros_msg
        t = m.log_time_ns / 1e9
        if topic == odom_topic:
            p = msg.pose.pose
            odom.append((t, p.position.x, p.position.y, quat_yaw(p.orientation)))
        elif topic == cm_topic:
            data = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width)
            costmaps.append({
                "t": t, "data": data,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "res": msg.info.resolution,
                "frame": msg.header.frame_id,
            })
        elif topic == coll_topic:
            coll.append((t, int(msg.action_type)))
        elif topic == plan_topic:
            if t - last_plan_t < 1.0:
                continue
            last_plan_t = t
            poses = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
            if poses:
                plans.append((t, msg.header.frame_id, poses))
    return odom, costmaps, coll, plans


def _render_one(out_path: Path, cm: dict, rx: float, ry: float, ryaw: float,
                segs: list, wp_enu: list, odom: list,
                plan_to_draw: list | None,
                t_rel: float, label: str, view_w: float = 20.0):
    """Render a single global_costmap snapshot view."""
    data = cm["data"].astype(np.float32)
    plot_data = np.where(data < 0, np.nan, data)  # -1 unknown → NaN
    H_cells, W_cells = data.shape
    x0 = cm["origin_x"]; y0 = cm["origin_y"]; res = cm["res"]
    extent = [x0, x0 + W_cells * res, y0, y0 + H_cells * res]

    fig, ax = plt.subplots(figsize=(11, 11))
    masked = np.ma.array(plot_data, mask=np.isnan(plot_data))
    COSTMAP_CMAP.set_bad((0.93, 0.93, 0.93))
    im = ax.imshow(masked, extent=extent, origin="lower", cmap=COSTMAP_CMAP,
                   vmin=0, vmax=100, alpha=0.95, aspect="equal",
                   interpolation="nearest")
    plt.colorbar(im, ax=ax, label="costmap value (0-100)", fraction=0.04)

    for ((ax_, ay_), (bx_, by_)) in segs:
        ax.plot([ax_, bx_], [ay_, by_], "--", color="#2c8a3e",
                linewidth=2, alpha=0.85)
    ax.plot([r[1] for r in odom], [r[2] for r in odom],
            color="#1565c0", linewidth=0.8, alpha=0.6)

    if plan_to_draw and len(plan_to_draw) >= 2:
        ax.plot([p[0] for p in plan_to_draw], [p[1] for p in plan_to_draw],
                color="#d61680", linewidth=2.5, alpha=0.95, label="active plan")

    for i, (x, y) in enumerate(wp_enu):
        if x < rx - view_w or x > rx + view_w: continue
        if y < ry - view_w or y > ry + view_w: continue
        ax.plot(x, y, "o", color="#1a5e2c", markersize=8,
                markeredgecolor="white", markeredgewidth=1.5)
        ax.annotate(f"WP{i}", (x, y), xytext=(6, 4),
                    textcoords="offset points", fontsize=10,
                    color="#1a5e2c", weight="bold")
    ax.plot(rx, ry, "X", color="#0a8a3e", markersize=18,
            markeredgecolor="white", markeredgewidth=2,
            label=f"robot @ t={t_rel:.1f}s, yaw={math.degrees(ryaw):.0f}°")
    arrow_len = 1.5
    ax.annotate("", xy=(rx + arrow_len*math.cos(ryaw),
                        ry + arrow_len*math.sin(ryaw)),
                xytext=(rx, ry),
                arrowprops=dict(arrowstyle="->", color="#0a8a3e", lw=2))

    ax.set_xlim(rx - view_w, rx + view_w)
    ax.set_ylim(ry - view_w, ry + view_w)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title(f"Actual global_costmap at t={t_rel:.1f}s ({label}) "
                 f"— {view_w*2:.0f}m view")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.2)
    plt.tight_layout()
    plt.savefig(out_path, dpi=140)
    plt.close(fig)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Render actual recorded global_costmap PNGs "
                    "at moments of interest")
    p.add_argument("bag_dir", type=Path)
    p.add_argument("--site-yaml", type=Path, default=None)
    p.add_argument("--mission-yaml", type=Path, default=None,
                   help="optional — adds segment + waypoint overlay")
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--prefix", default="/grunt1")
    p.add_argument("--at", type=float, action="append", default=[],
                   help="seconds-from-bag-start to render. Repeatable. "
                        "If omitted (or in addition), renders at each fresh "
                        "collision onset.")
    p.add_argument("--view-width-m", type=float, default=20.0,
                   help="size of square view around robot (default 20)")
    p.add_argument("--skip-fresh-onsets", action="store_true",
                   help="don't auto-render at fresh onsets")
    args = p.parse_args(argv)

    anchor = None
    segs = []
    wp_enu = []
    if args.mission_yaml:
        site_yaml = args.site_yaml or default_site_yaml(args.bag_dir)
        if site_yaml is None:
            print("warning: no --site-yaml; segment overlay disabled",
                  file=sys.stderr)
        else:
            anchor = GeoAnchor.from_site_yaml(site_yaml)
            wp_enu, segs = load_mission_segments(args.mission_yaml, anchor)

    print(f"Streaming {args.bag_dir} ...", file=sys.stderr)
    odom, costmaps, coll, plans = _stream_bag(args.bag_dir, prefix=args.prefix)
    if not odom or not costmaps:
        print("error: bag missing odometry or global_costmap topic",
              file=sys.stderr)
        return 1
    t0 = odom[0][0]
    odom_t = [r[0] for r in odom]
    cm_t = [c["t"] for c in costmaps]

    # Compute target moments
    targets = []
    if not args.skip_fresh_onsets:
        for (t, s) in fresh_onsets(coll):
            targets.append((t, f"fresh_{int(t-t0):03d}s"))
    for at in args.at:
        targets.append((t0 + at, f"at_{int(at):03d}s"))

    if not targets:
        print("warning: no targets (no fresh onsets, no --at)",
              file=sys.stderr)
        return 0

    out_dir = args.out or args.bag_dir
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    label = args.bag_dir.name

    for (t, tag) in targets:
        i = bisect.bisect_right(cm_t, t)
        if i == 0:
            continue
        cm = costmaps[i - 1]
        oi = bisect.bisect_left(odom_t, t)
        if oi >= len(odom):
            continue
        r = odom[oi]
        # Find latest plan in prior 2s, transform to map
        plan_map = None
        for (pt, frame, poses_bl) in reversed(plans):
            if pt > t:
                continue
            if t - pt > 2.0:
                break
            plan_map = transform_plan_to_map(poses_bl, frame, pt, odom, odom_t)
            break
        out_path = out_dir / f"actual_costmap_{tag}_{label}.png"
        _render_one(out_path, cm, r[1], r[2], r[3], segs, wp_enu,
                    odom, plan_map, t - t0, tag,
                    view_w=args.view_width_m)
        print(f"wrote {out_path.name}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
