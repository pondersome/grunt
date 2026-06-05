#!/usr/bin/env bash
# Installs and enables the grunt-wifi-on boot safety-net service.
# Run with: sudo bash ~/ros2_ws/install-grunt-wifi-on.sh
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)/grunt-wifi-on.service"
install -m 644 "$SRC" /etc/systemd/system/grunt-wifi-on.service
systemctl daemon-reload
systemctl enable --now grunt-wifi-on.service
systemctl status grunt-wifi-on.service --no-pager
