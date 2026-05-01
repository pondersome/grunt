"""Visualization — SVG plots for mission analyses.

Default backend is SVG so output is resolution-independent. Each plot
function returns the file path it wrote to.
"""
from __future__ import annotations

import bisect
import math
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("SVG")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from .geo import GeoAnchor, lla_to_local_enu


def accuracy_tube(parsed_by_label: dict[str, dict],
                   anchor: GeoAnchor,
                   waypoints_by_label: dict[str, list[tuple[float, float]]],
                   out_path: str | Path,
                   title: str = "Mission accuracy tube") -> Path:
    """Side-by-side accuracy tube panels for one or more missions.

    Args:
        parsed_by_label: dict mapping a panel label → output of `bag.load_bag`.
        anchor: site GeoAnchor for ENU conversion.
        waypoints_by_label: dict mapping panel label → list of (lat, lon) wp tuples.
            Use {} if no waypoint overlay desired.
        out_path: where to write the SVG.
        title: figure-level title.
    """
    n = len(parsed_by_label)
    if n == 0:
        raise ValueError("parsed_by_label is empty")
    fig, axes = plt.subplots(1, n, figsize=(7.5 * n, 12),
                              gridspec_kw={"wspace": 0.15})
    if n == 1:
        axes = [axes]
    fig.suptitle(title, fontsize=11)

    color_map = {0: "#cc0000", 1: "#ff8800", 2: "#00aa44"}  # NONE, FLOAT, FIXED

    for ax, (label, parsed) in zip(axes, parsed_by_label.items()):
        fix = parsed.get("/grunt1/rtk/fix_gated", [])
        nav = parsed.get("/grunt1/rtk/navpvt", [])
        odom = parsed.get("/grunt1/odometry/global", [])
        joy = parsed.get("/grunt1/cmd_vel_joy", [])

        # Pair fix samples with nearest navpvt for h_acc/carrier
        nav_t = [r[0] for r in nav]
        fix_pts = []  # (x, y, h_acc, carrier)
        for (t, lat, lon, _) in fix:
            x, y = lla_to_local_enu(lat, lon, anchor)
            i = bisect.bisect_left(nav_t, t)
            if i < len(nav):
                _, _, _, _, c, ha = nav[i]
            else:
                c, ha = 1, 0.05
            fix_pts.append((x, y, ha, c))

        # GPS path colored by carrier
        for i in range(len(fix_pts) - 1):
            x1, y1, _, c = fix_pts[i]
            x2, y2, _, _ = fix_pts[i + 1]
            ax.plot([x1, x2], [y1, y2], color=color_map.get(c, "#888"),
                    linewidth=0.8, alpha=0.7, zorder=2)

        # h_acc tubes (every Nth, scaled 4× for visibility)
        step = max(1, len(fix_pts) // 200)
        for i in range(0, len(fix_pts), step):
            x, y, ha, c = fix_pts[i]
            r = max(ha, 0.02) * 4
            ax.add_patch(Circle((x, y), r, facecolor=color_map.get(c, "#888"),
                                 alpha=0.08, edgecolor="none", zorder=1))

        # EKF path
        ekf_x = [r[1] for r in odom[::5]]
        ekf_y = [r[2] for r in odom[::5]]
        ax.plot(ekf_x, ekf_y, color="#666", linewidth=0.4, alpha=0.5,
                zorder=3, label="EKF /odometry/global")

        # Waypoints
        wp_lla = waypoints_by_label.get(label, [])
        if wp_lla:
            wp_xy = [lla_to_local_enu(lat, lon, anchor) for (lat, lon) in wp_lla]
            wxs = [w[0] for w in wp_xy]
            wys = [w[1] for w in wp_xy]
            ax.scatter(wxs, wys, s=80, marker="o", facecolor="yellow",
                       edgecolor="black", linewidth=1.5, zorder=10,
                       label="waypoints")
            for idx, (wx, wy) in enumerate(wp_xy):
                ax.annotate(str(idx), (wx, wy), fontsize=7, ha="center",
                             va="center", zorder=11)
            ax.plot(wxs, wys, "b-", linewidth=0.8, alpha=0.6, zorder=4,
                    label="waypoint path")

        # Site anchor at origin
        ax.scatter([0], [0], s=120, marker="*", color="red",
                    edgecolor="black", linewidth=1, zorder=12, label="site anchor")

        # Intervention markers (start of each joy burst)
        odom_t = [r[0] for r in odom]
        bursts = _detect_joy_bursts(joy)
        for bs, _ in bursts:
            i = bisect.bisect_left(odom_t, bs)
            if i < len(odom):
                ax.scatter([odom[i][1]], [odom[i][2]], s=180, marker="*",
                            color="red", edgecolor="black", linewidth=0.7,
                            zorder=15, alpha=0.9)

        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_title(label, fontsize=10)
        ax.set_xlabel("x (m, ENU at site anchor)")
        if ax is axes[0]:
            ax.set_ylabel("y (m, ENU at site anchor)")
        ax.legend(loc="upper left", fontsize=7)

    fig.text(0.5, 0.92,
              "GPS color: green=FIXED  orange=FLOAT  red=NONE     "
              "h_acc tube radius scaled 4× for visibility",
              ha="center", fontsize=9)

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, format="svg", bbox_inches="tight")
    plt.close(fig)
    return out


def _detect_joy_bursts(joy_samples: list[tuple],
                        min_burst_s: float = 0.5) -> list[tuple[float, float]]:
    """Return list of (start_t_abs, duration) for sustained joy commands."""
    bursts = []
    in_burst = False
    bs = None
    for (t, vx, vz) in joy_samples:
        active = abs(vx) > 0.01 or abs(vz) > 0.01
        if active and not in_burst:
            bs = t
            in_burst = True
        elif not active and in_burst:
            if t - bs >= min_burst_s:
                bursts.append((bs, t - bs))
            in_burst = False
    return bursts
