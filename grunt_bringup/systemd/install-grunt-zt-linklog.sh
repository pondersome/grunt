#!/usr/bin/env bash
# Installs the grunt-zt-linklog diagnostic logger service (does NOT enable/start it).
# Run with: sudo bash install-grunt-zt-linklog.sh
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)/grunt-zt-linklog.service"
install -m 644 "$SRC" /etc/systemd/system/grunt-zt-linklog.service
systemctl daemon-reload

cat <<'EOF'
Installed grunt-zt-linklog.service (not started, not enabled).

Use while on the hotspot:
  sudo systemctl start  grunt-zt-linklog      # begin logging
  sudo systemctl stop   grunt-zt-linklog      # stop
  systemctl status      grunt-zt-linklog
  journalctl -u grunt-zt-linklog -f           # watch it run

CSV: /home/karim/zt_linklog.csv  (one row per LEAF peer per 30s cycle)
Optional: add `--overlay-target 10.147.20.X` to ExecStart to also ping a peer overlay.
Multi-day study across reboots:  sudo systemctl enable grunt-zt-linklog
EOF
