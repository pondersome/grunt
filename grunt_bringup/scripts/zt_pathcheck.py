#!/usr/bin/env python3
"""
Settled ZeroTier path checker — distinguishes LIVE paths from stale-but-active ones.

Takes two `zerotier-cli -j peers` samples 12s apart and reports, per peer, the
preferred path's address family and whether its lastReceive is BOTH fresh (<8s)
and ADVANCING between samples. `active:true` alone is not trusted — ZeroTier keeps
expired-but-not-yet-reaped paths flagged active.

Run AFTER letting the link settle (>=60s post network switch), from the ws root:
    sudo python3 src/grunt/grunt_bringup/scripts/zt_pathcheck.py
"""
import json, subprocess, time

FRESH_S = 8.0  # a live keepalive'd path receives well within this

def snap():
    out = subprocess.check_output(["zerotier-cli", "-j", "peers"])
    return json.loads(out), time.time() * 1000.0

def fam(addr):
    return "v6" if ":" in addr.rsplit("/", 1)[0] else "v4"

s1, _ = snap()
print("sampling (12s)...")
time.sleep(12)
s2, t2 = snap()

prev = {}
for p in s1:
    for pa in p.get("paths", []):
        prev[(p["address"], pa["address"])] = pa["lastReceive"]

print(f"\n{'peer':<12} {'role':<7} {'preferred path':<46} {'fam':<4} {'rx_age':>7} {'adv':<5} state")
print("-" * 92)
for p in sorted(s2, key=lambda x: x.get("role", "")):
    role = p.get("role", "")
    paths = p.get("paths", [])
    pref = next((pa for pa in paths if pa.get("preferred")), None)
    if pref is None and paths:
        pref = max(paths, key=lambda x: x["lastReceive"])
    if pref is None:
        print(f"{p['address']:<12} {role:<7} (no paths)")
        continue
    age = (t2 - pref["lastReceive"]) / 1000.0
    adv = pref["lastReceive"] > prev.get((p["address"], pref["address"]), 0)
    state = "LIVE" if (age < FRESH_S and adv) else "stale"
    print(f"{p['address']:<12} {role:<7} {pref['address']:<46} {fam(pref['address']):<4} {age:6.1f}s {str(adv):<5} {state}")

print("\nLEAF (real machine) peers with NO live direct path => RELAYING via a root:")
relay = False
for p in s2:
    if p.get("role") != "LEAF":
        continue
    live = [pa for pa in p.get("paths", [])
            if (t2 - pa["lastReceive"]) / 1000.0 < FRESH_S
            and pa["lastReceive"] > prev.get((p["address"], pa["address"]), 0)]
    if not live:
        print(f"  {p['address']}  <-- RELAYING")
        relay = True
if not relay:
    print("  (none — every LEAF has a live direct path)")
