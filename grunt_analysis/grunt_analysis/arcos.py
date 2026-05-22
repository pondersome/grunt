"""ARCOS chassis stall / dropout audit.

Finds windows where /cmd_vel commands motion but the chassis isn't
delivering it (the "descending beep, robot stops and freewheels" fault),
and correlates them with the chassis telemetry added 2026-05-20:

  - /motor_stall   — firmware per-wheel stall flag (SIP lwstall/rwstall)
  - /motor_state   — ARCOS motor-enable bit
  - /battery_state — pack voltage

A "stall" here is inferred: cmd_vel asks for motion, /odometry/local
shows the chassis essentially still for >= a threshold duration. The
firmware /motor_stall flag is the ground-truth corroboration — but note
the flag is per-wheel while the physical fault (a straight roll-back on
a slope) implies BOTH motors briefly lose drive regardless of which
wheel the firmware flags.

Outputs:
- arcos_audit.md — battery / motor-state / stall-flag summary, the
  firmware stall-flag episodes, and the cmd-vs-chassis stall episodes
  with per-episode telemetry correlation + mission-thirds breakdown.

Run via:
    python -m grunt_analysis.arcos <bag_dir>
"""
from __future__ import annotations

import argparse
import bisect
import sys
from pathlib import Path

from .bag import load_bag, time_origin

# detection thresholds
CMD_MOVE_VX = 0.05      # m/s — cmd "wants to translate"
CMD_MOVE_VYAW = 0.15    # rad/s — cmd "wants to rotate"
ACT_STILL_VX = 0.03     # m/s — actual "not translating"
ACT_STILL_VYAW = 0.08   # rad/s — actual "not rotating"
MIN_EPISODE_S = 1.0     # ignore sub-second decel tails / cmd latency
MATCH_WIN = 0.20        # s — nearest-sample match window

TOPICS = ['/grunt1/cmd_vel', '/grunt1/wheel_cmd', '/grunt1/odometry/local',
          '/grunt1/nav/is_rotating_to_heading', '/grunt1/cmd_vel_joy',
          '/grunt1/motor_stall', '/grunt1/motor_state',
          '/grunt1/battery_state']

TRACK_M = 0.40          # P3-AT wheel track — for cmd_vel-derived per-wheel
WHEEL_MOVE = 0.03       # m/s — a wheel is "commanded to move" above this


def _nearest(times, table, t):
    """Nearest table row to time t, within MATCH_WIN; else None."""
    i = bisect.bisect_left(times, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(table):
            if best is None or abs(table[j][0] - t) < abs(best[0] - t):
                best = table[j]
    return best if best and abs(best[0] - t) <= MATCH_WIN else None


def detect_stall_episodes(cmd, odo, min_episode_s):
    """Episodes where cmd wants motion but odometry shows the chassis
    still, sustained >= min_episode_s. Each episode is a list of
    (t, stalled, cmd_vx, cmd_vyaw, act_vx, act_vyaw)."""
    odo_t = [r[0] for r in odo]
    flags = []
    for (t, cvx, cvyaw) in cmd:
        o = _nearest(odo_t, odo, t)
        if o is None:
            continue
        _, avx, avyaw = o
        wants = abs(cvx) > CMD_MOVE_VX or abs(cvyaw) > CMD_MOVE_VYAW
        still = abs(avx) < ACT_STILL_VX and abs(avyaw) < ACT_STILL_VYAW
        flags.append((t, wants and still, cvx, cvyaw, avx, avyaw))
    episodes = []
    i, n = 0, len(flags)
    while i < n:
        if flags[i][1]:
            j = i
            while j < n and flags[j][1]:
                j += 1
            if flags[j - 1][0] - flags[i][0] >= min_episode_s:
                episodes.append(flags[i:j])
            i = j
        else:
            i += 1
    return episodes


def firmware_stall_episodes(stall):
    """Spans where motor_stall left or right is set. Each: (t, dur, L, R)."""
    eps = []
    i, n = 0, len(stall)
    while i < n:
        if stall[i][1] or stall[i][2]:
            j = i
            while j < n and (stall[j][1] or stall[j][2]):
                j += 1
            seg = stall[i:j]
            eps.append((seg[0][0], seg[-1][0] - seg[0][0],
                        any(s[1] for s in seg), any(s[2] for s in seg)))
            i = j
        else:
            i += 1
    return eps


def perwheel_audit(stall, wheelcmd, cmd, mstate):
    """Per-wheel commanded-vs-measured velocity.

    Measured = motor_stall left_vel/right_vel (encoder, controller-
    independent). Commanded = wheel_cmd directly if present (the VEL2
    tank-drive bench test), else derived from cmd_vel (v_l/r = vx -/+
    w*track/2).

    Only motors-enabled samples count — VEL2 moves nothing with the
    motors off, so the Phase-A hand-spin check is excluded here (it is
    read off motor_stall directly). Tracking is a *magnitude* ratio
    sum|meas| / sum|cmd|, so forward+reverse runs do not cancel; a
    separate dir-agreement fraction catches a wheel running backwards.
    The weak drive channel shows the lower tracking ratio.

    Returns a dict, or None when there is nothing to compare.
    """
    if not stall or stall[0][3] is None or stall[0][4] is None:
        return None      # pre-2026-05-21 bag: no per-wheel velocity
    if wheelcmd:
        source = "wheel_cmd (VEL2 direct per-wheel)"
        src, src_t = wheelcmd, [c[0] for c in wheelcmd]

        def cmd_lr(row):
            return (row[1], row[2])
    elif cmd:
        source = "cmd_vel derived (v_l/r = vx -/+ w*track/2)"
        src, src_t = cmd, [c[0] for c in cmd]

        def cmd_lr(row):
            return (row[1] - row[2] * TRACK_M / 2,
                    row[1] + row[2] * TRACK_M / 2)
    else:
        return None

    ms_t = [m[0] for m in mstate]

    def motors_on(t):
        if not mstate:
            return True
        i = bisect.bisect_right(ms_t, t) - 1
        return i >= 0 and mstate[i][1] == 1

    pairs = []   # (cmd_l, cmd_r, meas_l, meas_r)
    n_motors_off = 0
    for s in stall:
        c = _nearest(src_t, src, s[0])
        if c is None:
            continue
        if not motors_on(s[0]):
            n_motors_off += 1
            continue
        cl, cr = cmd_lr(c)
        pairs.append((cl, cr, s[3], s[4]))
    if not pairs:
        return None

    def wheel(ci, mi, sub=None):
        pp = pairs if sub is None else [p for p in pairs if sub(p)]
        mv = [p for p in pp if abs(p[ci]) > WHEEL_MOVE]
        if not mv:
            return None
        sac = sum(abs(p[ci]) for p in mv)
        sam = sum(abs(p[mi]) for p in mv)
        return {'n': len(mv),
                'mean_cmd': sac / len(mv),     # mean magnitude
                'mean_meas': sam / len(mv),
                'tracking': sam / sac if sac > 1e-6 else float('nan'),
                'dir_ok': sum(1 for p in mv
                              if (p[ci] >= 0) == (p[mi] >= 0)) / len(mv)}

    out = {'source': source, 'n': len(pairs),
           'n_motors_off': n_motors_off,
           'left': wheel(0, 2), 'right': wheel(1, 3)}
    # one-wheel-isolated: only left commanded (cmd_r idle), and vice versa
    il = wheel(0, 2, lambda p: abs(p[1]) < WHEEL_MOVE)
    ir = wheel(1, 3, lambda p: abs(p[0]) < WHEEL_MOVE)
    if il:
        leak = [abs(p[3]) for p in pairs
                if abs(p[0]) > WHEEL_MOVE and abs(p[1]) < WHEEL_MOVE]
        il['idle_meas'] = sum(leak) / len(leak) if leak else 0.0
    if ir:
        leak = [abs(p[2]) for p in pairs
                if abs(p[1]) > WHEEL_MOVE and abs(p[0]) < WHEEL_MOVE]
        ir['idle_meas'] = sum(leak) / len(leak) if leak else 0.0
    out['iso_left'], out['iso_right'] = il, ir
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="ARCOS chassis stall audit")
    p.add_argument("bag_dir", help="bag directory or .mcap file")
    p.add_argument("--out", help="output dir (default: next to the bag)")
    p.add_argument("--min-episode", type=float, default=MIN_EPISODE_S,
                   help=f"min stall episode duration s (default {MIN_EPISODE_S})")
    args = p.parse_args(argv)
    min_episode = args.min_episode

    bag_dir = Path(args.bag_dir)
    out_dir = Path(args.out) if args.out else (
        bag_dir if bag_dir.is_dir() else bag_dir.parent)
    label = bag_dir.name

    d = load_bag(bag_dir, topics=TOPICS)
    t0 = time_origin(d)

    cmd = sorted((t - t0, vx, vyaw) for (t, vx, vyaw) in d['/grunt1/cmd_vel'])
    odo = sorted((t - t0, vx, vyaw)
                 for (t, yaw, vx, vyaw, x, y) in d['/grunt1/odometry/local'])
    rot = sorted((t - t0, f)
                 for (t, f) in d['/grunt1/nav/is_rotating_to_heading'])
    stall = sorted((t - t0, L, R, lv, rv)
                   for (t, L, R, lv, rv) in d['/grunt1/motor_stall'])
    wheelcmd = sorted((t - t0, l, r) for (t, l, r) in d['/grunt1/wheel_cmd'])
    mstate = sorted((t - t0, s) for (t, s) in d['/grunt1/motor_state'])
    batt = sorted((t - t0, v) for (t, v) in d['/grunt1/battery_state'])

    if not cmd and not wheelcmd:
        print(f"no /cmd_vel or /wheel_cmd in {label} — nothing to audit",
              file=sys.stderr)
        return 1
    dur = max(seq[-1][0] for seq in (cmd, wheelcmd, stall) if seq)
    rot_t = [r[0] for r in rot]
    stall_t = [s[0] for s in stall]
    mstate_t = [m[0] for m in mstate]
    batt_t = [b[0] for b in batt]

    def rotating_at(t):
        i = bisect.bisect_right(rot_t, t) - 1
        return rot[i][1] if 0 <= i < len(rot) else False

    def stall_flag_window(ts, te):
        lo = bisect.bisect_left(stall_t, ts - 0.5)
        hi = bisect.bisect_right(stall_t, te + 0.5)
        L = any(stall[i][1] for i in range(lo, hi))
        R = any(stall[i][2] for i in range(lo, hi))
        return L, R

    episodes = detect_stall_episodes(cmd, odo, min_episode)
    fw = firmware_stall_episodes(stall)

    L = ["# ARCOS chassis audit — " + label, ""]
    L.append(f"- duration: {dur:.0f}s")
    L.append(f"- samples: cmd_vel {len(cmd)}, odometry/local {len(odo)}, "
             f"motor_stall {len(stall)}, motor_state {len(mstate)}, "
             f"battery {len(batt)}")
    L.append("")

    # battery
    L.append("## Battery")
    if batt:
        vs = [b[1] for b in batt]
        L.append(f"- voltage: start {vs[0]:.2f} V, end {vs[-1]:.2f} V, "
                 f"min {min(vs):.2f} V, max {max(vs):.2f} V")
    else:
        L.append("- no /battery_state in bag")
    L.append("")

    # motor enable state
    L.append("## Motor enable state (motor_state)")
    if mstate:
        zeros = [m for m in mstate if m[1] == 0]
        L.append(f"- {len(mstate)} samples, {len(zeros)} with state==0")
        if zeros:
            L.append(f"- first reported disabled at t={zeros[0][0]:.1f}s")
        else:
            L.append("- ARCOS never reported the motors disabled — the "
                     "fault does not clear the enable bit, so motor_state "
                     "alone cannot see it")
    else:
        L.append("- no /motor_state in bag")
    L.append("")

    # firmware stall flag
    L.append("## Firmware stall flag (motor_stall)")
    if stall:
        sl = sum(1 for s in stall if s[1])
        sr = sum(1 for s in stall if s[2])
        L.append(f"- left set: {sl}/{len(stall)} samples")
        L.append(f"- right set: {sr}/{len(stall)} samples")
        L.append("")
        L.append(f"### Firmware stall-flag episodes: {len(fw)}")
        if fw:
            L.append("")
            L.append("| # | t_start | dur_s | wheels |")
            L.append("|---|---------|-------|--------|")
            for k, (ts, ed, fl, fr) in enumerate(fw):
                w = ('L' if fl else '-') + ('R' if fr else '-')
                L.append(f"| {k} | {ts:.1f} | {ed:.1f} | {w} |")
    else:
        L.append("- no /motor_stall in bag (pre-2026-05-20 driver?)")
    L.append("")

    # per-wheel velocity audit (the bench test: commanded vs measured)
    pw = perwheel_audit(stall, wheelcmd, cmd, mstate)
    L.append("## Per-wheel velocity audit")
    if pw is None:
        L.append("- no per-wheel data — needs /motor_stall carrying "
                 "left_vel/right_vel (driver >= 2026-05-21) plus a "
                 "/wheel_cmd or /cmd_vel command stream")
    else:
        L.append(f"- command source: {pw['source']}")
        L.append(f"- measured: motor_stall left_vel/right_vel (encoder, "
                 f"controller-independent); {pw['n']} motors-enabled "
                 f"paired samples ({pw['n_motors_off']} motors-off "
                 f"samples excluded)")
        L.append("- tracking = sum|measured| / sum|commanded| over "
                 "moving samples; dir = fraction with measured sign "
                 "matching command")
        L.append("")
        L.append("| wheel | n moving | mean |cmd| | mean |meas| "
                 "| tracking | dir |")
        L.append("|---|---|---|---|---|---|")
        for w in ('left', 'right'):
            r = pw[w]
            if r:
                tk = (f"{100 * r['tracking']:.0f}%"
                      if r['tracking'] == r['tracking'] else "n/a")
                L.append(f"| {w} | {r['n']} | {r['mean_cmd']:.3f} | "
                         f"{r['mean_meas']:.3f} | {tk} | "
                         f"{100 * r['dir_ok']:.0f}% |")
            else:
                L.append(f"| {w} | 0 | — | — | — | — |")
        lr, rr = pw['left'], pw['right']
        if lr and rr and lr['tracking'] == lr['tracking'] \
                and rr['tracking'] == rr['tracking']:
            gap = abs(lr['tracking'] - rr['tracking'])
            weak = 'left' if lr['tracking'] < rr['tracking'] else 'right'
            lo, hi = sorted((lr['tracking'], rr['tracking']))
            L.append("")
            if gap > 0.10:
                L.append(f"- **Asymmetry: the {weak} wheel tracks "
                         f"{100 * lo:.0f}% of its command vs "
                         f"{100 * hi:.0f}% on the other — the {weak} "
                         f"drive channel (motor / encoder / H-bridge) is "
                         f"the weak side.**")
            else:
                L.append(f"- Wheels track within {100 * gap:.0f}% of each "
                         f"other — no per-wheel asymmetry in this bag.")
        for w in ('left', 'right'):
            r = pw[w]
            if r and r['dir_ok'] < 0.9:
                L.append(f"- **{w} wheel: measured sign disagrees with "
                         f"command {100 * (1 - r['dir_ok']):.0f}% of the "
                         f"time — possible reversed encoder/wiring or a "
                         f"wheel not following the command.**")
        if pw['iso_left'] or pw['iso_right']:
            L.append("")
            L.append("### One-wheel-isolated segments (cleanest signal)")
            L.append("")
            L.append("| driven wheel | n | mean |cmd| | mean |meas| | "
                     "tracking | idle-wheel |meas| |")
            L.append("|---|---|---|---|---|---|")
            for w in ('left', 'right'):
                r = pw[f'iso_{w}']
                if r:
                    tk = (f"{100 * r['tracking']:.0f}%"
                          if r['tracking'] == r['tracking'] else "n/a")
                    L.append(f"| {w} | {r['n']} | {r['mean_cmd']:.3f} | "
                             f"{r['mean_meas']:.3f} | {tk} | "
                             f"{r.get('idle_meas', 0.0):.3f} |")
    L.append("")

    # cmd-vs-chassis stall episodes
    L.append(f"## cmd-vs-chassis stall episodes (>= {min_episode}s): "
             f"{len(episodes)}")
    total = 0.0
    if episodes:
        L.append("")
        L.append("| # | t_start | dur_s | cmd_vyaw | rot% | fw_flag | "
                 "motor_state | batt_V |")
        L.append("|---|---------|-------|----------|------|---------|"
                 "-------------|--------|")
        for k, seg in enumerate(episodes):
            ts = seg[0][0]
            ed = seg[-1][0] - seg[0][0]
            total += ed
            cvyaw = sum(abs(s[3]) for s in seg) / len(seg)
            rf = sum(1 for s in seg if rotating_at(s[0])) / len(seg)
            fl, fr = stall_flag_window(ts, seg[-1][0])
            fwstr = (('L' if fl else '-') + ('R' if fr else '-')
                     ) if stall else 'n/a'
            ms = _nearest(mstate_t, mstate, ts + ed / 2)
            msstr = str(ms[1]) if ms else 'n/a'
            bv = _nearest(batt_t, batt, ts)
            bvstr = f"{bv[1]:.2f}" if bv else 'n/a'
            L.append(f"| {k} | {ts:.1f} | {ed:.1f} | {cvyaw:.3f} | "
                     f"{100*rf:.0f}% | {fwstr} | {msstr} | {bvstr} |")
        L.append("")
        L.append(f"- total stalled: {total:.1f}s ({100*total/dur:.1f}% "
                 f"of mission)")
        if stall:
            caught = sum(1 for seg in episodes
                         if any(stall_flag_window(seg[0][0], seg[-1][0])))
            L.append(f"- firmware flag corroborates {caught}/{len(episodes)} "
                     f"episodes")
        # thirds
        L.append("")
        L.append("### By mission third")
        L.append("")
        L.append("| third | window | episodes | stalled_s |")
        L.append("|-------|--------|----------|-----------|")
        for k in range(3):
            lo, hi = k * dur / 3, (k + 1) * dur / 3
            se = [e for e in episodes if lo <= e[0][0] < hi]
            ss = sum(e[-1][0] - e[0][0] for e in se)
            L.append(f"| {k+1} | {lo:.0f}-{hi:.0f}s | {len(se)} | {ss:.1f} |")
    L.append("")

    out = out_dir / "arcos_audit.md"
    out.write_text("\n".join(L) + "\n")
    print(f"wrote {out}", file=sys.stderr)
    print(f"{label}: {len(episodes)} stall episodes, {total:.0f}s stalled; "
          f"firmware flag episodes {len(fw)}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
