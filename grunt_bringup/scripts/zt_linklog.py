#!/usr/bin/env python3
"""
ZeroTier-over-cellular link logger — longitudinal correlation study.

Every --interval seconds, fires three INDEPENDENT latency probes at the same
instant and reads ZeroTier's own per-peer view, appending one CSV row per LEAF
peer per cycle. The point is to answer two questions across many hotspot cycles:

  Q1  Is punch-through (direct path) actually better and consistent over time?
      -> the `state` (direct/relay) and `path_fam` columns per peer over time.

  Q2  Are bad ZeroTier latencies ALWAYS concurrent with bad transport latency?
      -> compare `zt_lat_ms` / `t_ovl_ms` against `t_net_ms` at the same ts.
         `t_net_ms` is the CONTROL: a ping that bypasses ZeroTier entirely.
         Both bad together => link-limited (ZT exonerated). ZT bad while t_net
         fine => a real punch-through/relay problem. `t_gw_ms` decomposes the
         link further (robot<->phone radio vs carrier backhaul).

Probes per cycle:
  t_gw   ping the default-route gateway (tether first hop, e.g. 192.168.43.1)
  t_net  ping a public IPv6 host (default Cloudflare) -- bypasses ZeroTier
  t_ovl  ping a peer OVERLAY ip (10.147.20.x), if --overlay-target given

Runs as root (for `zerotier-cli`); intended to be driven by the
grunt-zt-linklog.service systemd unit. Ctrl-C / SIGTERM flushes and exits clean.

  sudo python3 src/grunt/grunt_bringup/scripts/zt_linklog.py [--interval 30] \
       [--out /home/karim/zt_linklog.csv] [--overlay-target 10.147.20.20]
"""
import argparse, csv, json, os, re, signal, subprocess, sys, time
from datetime import datetime, timezone

FIELDS = [
    "run_id", "ts_utc", "net_ctx",
    "peer_id", "peer_role", "zt_lat_ms", "path_fam", "path_addr", "state",
    "t_gw_ms", "t_gw_loss", "t_net_ms", "t_net_loss", "t_ovl_ms", "t_ovl_loss",
    "n_leaf", "n_relay",
]
FRESH_MS = 30000.0  # a LEAF whose preferred path received within this = direct
_running = True


def _stop(*_):
    global _running
    _running = False


def sh(cmd, timeout=10):
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:
        return ""


def net_ctx():
    """Home-vs-hotspot discriminator: the /64 of the first non-overlay global v6."""
    out = sh(["ip", "-6", "addr", "show", "scope", "global"])
    for m in re.finditer(r"inet6\s+([0-9a-f:]+)/\d+", out):
        a = m.group(1)
        if not a.startswith("fd60"):          # skip the ZT overlay address
            return ":".join(a.split(":")[:4]) + "::/64"
    return "unknown"


def default_gw():
    out = sh(["ip", "-4", "route", "show", "default"])
    m = re.search(r"default\s+via\s+(\S+)", out)
    return m.group(1) if m else None


def ping(target, v6=False):
    """Return (avg_ms_or_'', loss_pct). target=None -> ('','')."""
    if not target:
        return "", ""
    cmd = ["ping", "-6"] if v6 else ["ping"]
    cmd += ["-c", "3", "-i", "0.3", "-W", "1", target]
    out = sh(cmd, timeout=8)
    loss = ""
    ml = re.search(r"(\d+(?:\.\d+)?)% packet loss", out)
    if ml:
        loss = ml.group(1)
    avg = ""
    ma = re.search(r"=\s*[\d.]+/([\d.]+)/", out)   # min/avg/max/mdev
    if ma:
        avg = ma.group(1)
    return avg, loss


def zt_peers():
    try:
        return json.loads(sh(["zerotier-cli", "-j", "peers"], timeout=8) or "[]")
    except Exception:
        return []


def fam(addr):
    return "v6" if ":" in addr.rsplit("/", 1)[0] else "v4"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=30.0)
    ap.add_argument("--out", default="/home/karim/zt_linklog.csv")
    ap.add_argument("--net-host", default="2606:4700:4700::1111")  # Cloudflare v6
    ap.add_argument("--overlay-target", default=None)              # e.g. 10.147.20.20
    args = ap.parse_args()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    # One id per process start, stamped on every row of this run (for GROUP BY).
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

    # If an existing CSV has a different (older) schema, rotate it aside so we
    # don't silently append misaligned columns.
    if os.path.exists(args.out) and os.path.getsize(args.out) > 0:
        with open(args.out) as rf:
            if rf.readline().rstrip("\n") != ",".join(FIELDS):
                os.rename(args.out, args.out + ".bak")

    new = not os.path.exists(args.out) or os.path.getsize(args.out) == 0
    f = open(args.out, "a", newline="")
    w = csv.DictWriter(f, fieldnames=FIELDS)
    if new:
        w.writeheader()
        f.flush()
    try:
        os.chmod(args.out, 0o644)
    except OSError:
        pass

    while _running:
        cycle_start = time.time()
        ts = datetime.now(timezone.utc).isoformat(timespec="seconds")
        ctx = net_ctx()
        gw = default_gw()
        t_gw_ms, t_gw_loss = ping(gw, v6=(gw and ":" in gw))
        t_net_ms, t_net_loss = ping(args.net_host, v6=True)
        t_ovl_ms, t_ovl_loss = ping(args.overlay_target,
                                    v6=(args.overlay_target and ":" in args.overlay_target))

        peers = zt_peers()
        now = time.time() * 1000.0
        leaves = [p for p in peers if p.get("role") == "LEAF"]
        rows, n_relay = [], 0
        for p in leaves:
            paths = p.get("paths", [])
            pref = next((x for x in paths if x.get("preferred")), None)
            if pref is None and paths:
                pref = max(paths, key=lambda x: x.get("lastReceive", 0))
            if pref is None:
                state, pf, pa = "relay", "", ""
                n_relay += 1
            else:
                fresh = (now - pref.get("lastReceive", 0)) < FRESH_MS
                state = "direct" if fresh else "relay"
                if state == "relay":
                    n_relay += 1
                pf, pa = fam(pref["address"]), pref["address"]
            rows.append({
                "peer_id": p.get("address", ""), "peer_role": "LEAF",
                "zt_lat_ms": p.get("latency", ""), "path_fam": pf,
                "path_addr": pa, "state": state,
            })

        common = {
            "run_id": run_id, "ts_utc": ts, "net_ctx": ctx,
            "t_gw_ms": t_gw_ms, "t_gw_loss": t_gw_loss,
            "t_net_ms": t_net_ms, "t_net_loss": t_net_loss,
            "t_ovl_ms": t_ovl_ms, "t_ovl_loss": t_ovl_loss,
            "n_leaf": len(leaves), "n_relay": n_relay,
        }
        if not rows:   # still log link probes even with no peers
            rows = [{"peer_id": "", "peer_role": "", "zt_lat_ms": "",
                     "path_fam": "", "path_addr": "", "state": ""}]
        for r in rows:
            r.update(common)
            w.writerow(r)
        f.flush()

        sleep = args.interval - (time.time() - cycle_start)
        # responsive to SIGTERM: sleep in small slices
        while _running and sleep > 0:
            time.sleep(min(0.5, sleep))
            sleep -= 0.5

    f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
