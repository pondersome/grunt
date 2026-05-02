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
from matplotlib.lines import Line2D

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
        carrots = parsed.get("/grunt1/nav/lookahead_point", [])
        rotating = parsed.get("/grunt1/nav/is_rotating_to_heading", [])

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

        # Carrot stream — published in base_link frame (robot body frame).
        # To overlay on the map-frame plot we rotate by the robot's yaw
        # and translate by the robot's map-frame position, both at the
        # carrot's timestamp. Source: /grunt1/odometry/global at the
        # matching time gives both yaw and position.
        if carrots and odom:
            odom_t_full = [r[0] for r in odom]
            cstep = max(1, len(carrots) // 400)
            cx, cy = [], []
            for (ct, ccx, ccy) in carrots[::cstep]:
                j = bisect.bisect_left(odom_t_full, ct)
                if j >= len(odom) or abs(odom[j][0] - ct) > 0.5:
                    continue
                # odom: (t, x, y, yaw_rad) — yaw already in radians.
                _, rx, ry, ryaw = odom[j]
                cos_y = math.cos(ryaw); sin_y = math.sin(ryaw)
                cx_w = rx + ccx * cos_y - ccy * sin_y
                cy_w = ry + ccx * sin_y + ccy * cos_y
                cx.append(cx_w); cy.append(cy_w)
            if cx:
                ax.scatter(cx, cy, s=4, color="#9333ea", marker=".",
                           alpha=0.6, zorder=5,
                           label=f"carrot ({len(carrots)} pts → map, "
                                 f"every {cstep}th shown)")

        # Rotation-mode highlight: where in space was the robot when RPP
        # was in rotate-to-heading mode? Plot EKF positions during those
        # episodes with a distinctive marker. Useful for "did RPP rotate
        # in place at every waypoint, or only the sharp ones?"
        if rotating and odom:
            odom_t_full = [r[0] for r in odom]
            rot_xs, rot_ys = [], []
            in_rot = False
            for (t, flag) in rotating:
                if flag and not in_rot:
                    in_rot = True
                    j = bisect.bisect_left(odom_t_full, t)
                    if j < len(odom):
                        rot_xs.append(odom[j][1]); rot_ys.append(odom[j][2])
                elif not flag and in_rot:
                    in_rot = False
            if rot_xs:
                ax.scatter(rot_xs, rot_ys, s=80, marker="P",
                           facecolor="#fbbf24", edgecolor="black",
                           linewidth=0.7, zorder=14,
                           label=f"rotate-to-heading start ({len(rot_xs)}×)")

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


def cmd_vs_actual(parsed: dict, out_path: str | Path,
                  title: str = "Cmd vs actual chassis velocity") -> Path:
    """Three-panel diagnostic for cmd-vel response health:

      Left  — angular velocity time series: cmd vs actual
      Mid   — angular cmd vs actual scatter, with y=x and regression line
      Right — linear cmd vs actual scatter

    Reveals lag (offset between traces in time series), gain reduction
    (regression slope < 1), and saturation (actual flatlines while cmd
    grows).
    """
    import bisect
    cmd = parsed.get("/grunt1/cmd_vel", [])
    local = parsed.get("/grunt1/odometry/local", [])
    if not cmd or not local or len(local[0]) < 4:
        raise ValueError("cmd_vs_actual: missing cmd_vel or odometry/local")

    odom_t = [r[0] for r in local]
    matched = []
    for (t, vx, vyaw) in cmd:
        i = bisect.bisect_left(odom_t, t)
        if i >= len(local) or abs(local[i][0] - t) > 0.05:
            continue
        # local: (t, yaw_rad, vx, vyaw, x, y)
        matched.append((t, vx, vyaw, local[i][2], local[i][3]))
    if not matched:
        raise ValueError("cmd_vs_actual: no time-aligned cmd/actual pairs")

    t0 = matched[0][0]
    ts = [r[0] - t0 for r in matched]
    cmd_lin = [r[1] for r in matched]
    cmd_ang = [r[2] for r in matched]
    actual_lin = [r[3] for r in matched]
    actual_ang = [r[4] for r in matched]

    fig = plt.figure(figsize=(18, 6))
    gs = fig.add_gridspec(1, 3, width_ratios=[2, 1, 1], wspace=0.25)
    fig.suptitle(title, fontsize=12)

    # ── Panel 1: angular time series ──
    ax1 = fig.add_subplot(gs[0, 0])
    # Plot a window of the middle-most third — full series at 10 Hz over
    # 5 minutes is too dense to see lag visually.
    n = len(ts)
    a, b = n // 3, n // 3 + min(n // 3, 600)  # ~60 s window
    ax1.plot(ts[a:b], cmd_ang[a:b], color="#dc2626", linewidth=1.0,
             label="commanded ω", alpha=0.9)
    ax1.plot(ts[a:b], actual_ang[a:b], color="#0891b2", linewidth=1.0,
             label="actual ω (odom)", alpha=0.9)
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("angular velocity (rad/s)")
    ax1.set_title(f"angular cmd vs actual — middle 60s window "
                  f"(t={ts[a]:.0f}…{ts[b-1]:.0f}s)", fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=9)
    ax1.axhline(0, color="black", linewidth=0.5, alpha=0.5)

    # ── Panel 2: angular scatter ──
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.scatter(cmd_ang, actual_ang, s=4, alpha=0.15, color="#0891b2")
    lim = max(abs(min(cmd_ang + actual_ang)),
              abs(max(cmd_ang + actual_ang))) * 1.05
    ax2.plot([-lim, lim], [-lim, lim], color="black",
             linewidth=1, linestyle="--", label="actual = cmd", alpha=0.6)
    # Regression on |cmd| > 0.1
    f = [(c, a) for c, a in zip(cmd_ang, actual_ang) if abs(c) > 0.1]
    if len(f) > 30:
        cs, as_ = zip(*f)
        cm = sum(cs) / len(cs); am = sum(as_) / len(as_)
        num = sum((cs[i] - cm) * (as_[i] - am) for i in range(len(cs)))
        den = sum((c - cm) ** 2 for c in cs)
        if den > 1e-9:
            slope = num / den
            intercept = am - slope * cm
            ax2.plot([-lim, lim],
                     [slope * -lim + intercept, slope * lim + intercept],
                     color="#dc2626", linewidth=1.5,
                     label=f"fit: slope={slope:.2f}")
    ax2.set_xlabel("commanded ω (rad/s)")
    ax2.set_ylabel("actual ω (rad/s)")
    ax2.set_xlim(-lim, lim); ax2.set_ylim(-lim, lim)
    ax2.set_title("angular cmd → actual", fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper left", fontsize=9)
    ax2.set_aspect("equal")

    # ── Panel 3: linear scatter ──
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.scatter(cmd_lin, actual_lin, s=4, alpha=0.15, color="#0891b2")
    lim_l = max(abs(min(cmd_lin + actual_lin)),
                abs(max(cmd_lin + actual_lin))) * 1.05
    ax3.plot([-lim_l, lim_l], [-lim_l, lim_l], color="black",
             linewidth=1, linestyle="--", label="actual = cmd", alpha=0.6)
    f = [(c, a) for c, a in zip(cmd_lin, actual_lin) if abs(c) > 0.05]
    if len(f) > 30:
        cs, as_ = zip(*f)
        cm = sum(cs) / len(cs); am = sum(as_) / len(as_)
        num = sum((cs[i] - cm) * (as_[i] - am) for i in range(len(cs)))
        den = sum((c - cm) ** 2 for c in cs)
        if den > 1e-9:
            slope = num / den
            intercept = am - slope * cm
            ax3.plot([-lim_l, lim_l],
                     [slope * -lim_l + intercept, slope * lim_l + intercept],
                     color="#dc2626", linewidth=1.5,
                     label=f"fit: slope={slope:.2f}")
    ax3.set_xlabel("commanded vx (m/s)")
    ax3.set_ylabel("actual vx (m/s)")
    ax3.set_xlim(-lim_l, lim_l); ax3.set_ylim(-lim_l, lim_l)
    ax3.set_title("linear cmd → actual", fontsize=10)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper left", fontsize=9)
    ax3.set_aspect("equal")

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, format="svg", bbox_inches="tight")
    plt.close(fig)
    return out
