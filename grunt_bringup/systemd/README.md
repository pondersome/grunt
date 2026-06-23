# grunt_bringup systemd units

System-level units for the Grunt robot (barney NUC). These are **not** ROS
nodes — they are host systemd services installed outside the ROS workspace.

## grunt-zt-linklog.service

ZeroTier-over-cellular **link logger** for a longitudinal correlation study —
not a boot service. Drives `scripts/zt_linklog.py` to append one CSV row per
LEAF peer every 30 s to `/home/karim/zt_linklog.csv`.

**Why:** to test, across many hotspot cycles, two claims that a single snapshot
can't prove — (1) that punch-through (direct paths) is genuinely better and
consistent, and (2) that bad ZeroTier latency is *always* concurrent with bad
underlying transport latency (i.e. the cellular link, not ZeroTier, is at fault).

Each cycle fires three independent latency probes at the same instant plus
ZeroTier's own per-peer view:
- `t_gw` — ping the default-route gateway (tether first hop, e.g. `192.168.43.1`):
  isolates the robot↔phone radio.
- `t_net` — ping a public IPv6 host (Cloudflare): **the control** — bypasses
  ZeroTier, so it measures pure transport.
- `t_ovl` — ping a peer overlay IP (only with `--overlay-target`): app-level.
- per LEAF peer: `zt_lat_ms`, `path_fam` (v4/v6), `state` (direct/relay via
  preferred-path freshness), plus cycle `n_leaf`/`n_relay`.

**Q2** is answered by comparing `zt_lat_ms`/`t_ovl_ms` against `t_net_ms` at the
same `ts_utc`: both bad together ⇒ link-limited; ZT bad while `t_net` fine ⇒ a
real ZT/punch-through problem. **Q1** by the `state`/`path_fam` columns over time.
Filter sessions with `net_ctx` (the `/64` prefix distinguishes home vs hotspot).

The CSV **appends across runs/reboots** (header written once). Every row carries
a `run_id` (the process-start timestamp) so runs can be told apart / grouped; a
time gap in `ts_utc` also marks a stop/start. If the schema ever changes, an
existing CSV with the old header is rotated to `<file>.bak` rather than corrupted.

### Install / use

    sudo bash install-grunt-zt-linklog.sh        # installs unit, does not start it
    sudo systemctl start  grunt-zt-linklog        # begin logging (while on hotspot)
    sudo systemctl stop   grunt-zt-linklog        # stop
    journalctl -u grunt-zt-linklog -f             # watch

Runs as root so `zerotier-cli` works without sudo. The installer deliberately
does **not** enable it at boot; `sudo systemctl enable grunt-zt-linklog` only if
a multi-day study should survive reboots. To also ping a peer overlay address,
add `--overlay-target 10.147.20.X` to `ExecStart` in the unit.

## grunt-wifi-on.service

Forces the WiFi radio on at every boot.

**Why:** NetworkManager persists the soft radio state in
`/var/lib/NetworkManager/NetworkManager.state` (`WirelessEnabled=false`). A WiFi
toggle-off from the desktop GUI (or `nmcli radio wifi off`) survives reboots and
leaves the headless robot unreachable — recovery otherwise requires opening the
chassis and attaching a monitor + keyboard. This is a soft NM state, not an
rfkill hardware block.

The unit is a oneshot ordered `After=NetworkManager.service`; it waits for NM
startup via `nm-online -s` (NB: `-s` waits for NM to be ready for D-Bus, **not**
for an active connection — without it the wait blocks the full timeout when wifi
is off, the exact case we must recover from, and a non-zero exit would skip the
radio-on), then runs `nmcli radio wifi on` (idempotent).

### Install

    sudo bash install-grunt-wifi-on.sh

Run from your own shell — `sudo` needs a real TTY for the password prompt.

### Verifying

WiFi is the robot's only regular access path, so any test that turns wifi off
risks stranding it if recovery fails. Two ways to test:

- **Reboot test (the only honest test of boot ordering):** do this *only* with a
  keyboard + monitor attached, so a failure costs nothing. `nmcli radio wifi off`,
  `sudo reboot`, then confirm `nmcli radio` shows `WIFI: enabled` and
  `systemctl status grunt-wifi-on.service` is `active (exited)`.
- **Remote no-reboot smoke-test:** `sudo bash wifi_selftest.sh`. Turns wifi off
  but schedules an *independent* systemd-timer backstop that unconditionally
  re-enables wifi after 120s, so a still-broken service can't strand the robot.
  Tests the command path only, not boot ordering.

**Verified 2026-06-04 (runtime/command path):** ran `wifi_selftest.sh` against a
genuine wifi-off state. Journal: wifi off at 22:34:23, `grunt-wifi-on.service`
restarted and Finished in the same second at 22:34:43 re-enabling wifi (the
same-second finish confirms `nm-online -s` returned immediately rather than
blocking 30s — the old `-t 30`-without-`-s` bug would have shown a 30s gap); the
120s backstop didn't fire until 22:36:24, long after wifi was already back, so it
was a no-op.

**Verified 2026-06-04 (boot ordering):** `nmcli radio wifi off` + reboot
(monitor-attached). After boot the unit ran 22:57:36 Starting → 22:57:42
Finished — a 6s gap showing `nm-online -s` waiting for NM startup (vs the
same-second warm-restart finish above), well inside the 30s timeout — then
`nmcli radio wifi on` exited `status=0/SUCCESS` and `nmcli radio` showed
`WIFI: enabled`. WiFi was disabled before reboot and restored unattended. **Both
paths (runtime/GUI-toggle and boot) are now verified against the real failure
state.**

### Trade-off

WiFi can no longer be *persistently* disabled; a reboot always re-enables it. To
genuinely keep it off across a reboot:

    sudo systemctl disable grunt-wifi-on.service
