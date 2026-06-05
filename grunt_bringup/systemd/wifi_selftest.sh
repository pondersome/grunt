#!/usr/bin/env bash
# Safe, remote-friendly self-test of grunt-wifi-on.service WITHOUT a reboot.
#
# The danger: testing a wifi-recovery mechanism means turning wifi off, which
# drops your SSH session. If recovery fails you are stranded (headless robot,
# wifi-only access). This script removes that risk with an INDEPENDENT backstop:
# a transient systemd timer that unconditionally re-enables wifi after
# BACKSTOP_SEC, regardless of whether grunt-wifi-on.service works or your SSH
# session survives. So the worst case is "wifi is back in BACKSTOP_SEC".
#
# Run from your own shell (needs root for nmcli radio / systemd-run):
#   sudo bash wifi_selftest.sh
#
# Then wait. Your SSH will drop. If grunt-wifi-on.service is healthy, wifi
# returns within OFF_WAIT+a few seconds and you reconnect quickly. If it is
# broken, the backstop brings wifi back at BACKSTOP_SEC. Afterwards read the log
# and check the unit:
#   cat /tmp/wifi_selftest.log
#   systemctl status grunt-wifi-on.service
set -euo pipefail

BACKSTOP_SEC=120   # unconditional wifi-on fires this long after launch
OFF_WAIT=20        # how long wifi stays off before we exercise the service
LOG=/tmp/wifi_selftest.log

# 1. Independent backstop -- runs under systemd init, survives SSH drop, does
#    NOT depend on grunt-wifi-on.service. This is what prevents stranding.
systemd-run --on-active="${BACKSTOP_SEC}" --unit=wifi-selftest-backstop --collect \
  /usr/bin/nmcli radio wifi on

# 2. The actual test, detached so it completes even after our SSH session dies.
systemd-run --unit=wifi-selftest-run --collect /bin/bash -c "
  {
    echo \"=== selftest start (backstop=${BACKSTOP_SEC}s, off_wait=${OFF_WAIT}s)\"
    echo 'disabling wifi...'; nmcli radio wifi off
    sleep ${OFF_WAIT}
    echo 'restarting grunt-wifi-on.service (the thing under test)...'
    systemctl restart grunt-wifi-on.service || echo 'SERVICE RESTART FAILED'
    sleep 3
    echo 'wifi state after service:'; nmcli radio
    echo '=== selftest end'
  } >>'${LOG}' 2>&1
"

echo "Self-test launched. Your SSH will drop in ~${OFF_WAIT}s."
echo "If grunt-wifi-on.service works, wifi returns shortly after that."
echo "If it does NOT, the backstop forces wifi on at ${BACKSTOP_SEC}s no matter what."
echo "After reconnecting:  cat ${LOG}  &&  systemctl status grunt-wifi-on.service"
