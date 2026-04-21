#!/usr/bin/env bash
# Stress-test the grunt systemd service start/stop cycle.
#
# Purpose: verify p2os_driver reliably re-syncs with ARCOS across
# back-to-back restarts. Used to validate shutdown-path changes and
# to isolate the firmware-watchdog-vs-clean-shutdown hypothesis by
# varying the interval between stop and start.
#
# Fast cadence (< ARCOS watchdog ~2s) exposes dirty-exit bugs:
#   test_restart_cycles.sh -n 10 -d 1
# Slow cadence (> watchdog) verifies firmware cleanup masks a dirty exit:
#   test_restart_cycles.sh -n 10 -d 5
#
# Options:
#   -n CYCLES  number of stop/start cycles (default: 10)
#   -d DELAY   seconds between stop and next start (default: 1)
#   -u UPTIME  max seconds to wait per cycle before giving up (default: 75)
#              Polls the journal every 3s; returns as soon as a
#              success or failure marker appears. 75s caps worst-case
#              wait for dirty-state startups that retry through the
#              2.5s ARCOS watchdog + baud-rate cycling before logging
#              "p2os setup failed".
#   -s NAME    systemd service name (default: grunt)
#
# Requires sudo for systemctl. Logs each cycle's outcome and
# prints a summary. Non-zero exit code if any cycle failed.

set -u

CYCLES=10
DELAY=1
UPTIME=75
SERVICE=grunt

usage() {
  sed -n '2,22p' "$0"
  exit "${1:-0}"
}

while getopts "n:d:u:s:h" opt; do
  case "$opt" in
    n) CYCLES=$OPTARG ;;
    d) DELAY=$OPTARG ;;
    u) UPTIME=$OPTARG ;;
    s) SERVICE=$OPTARG ;;
    h) usage 0 ;;
    *) usage 1 ;;
  esac
done

pass=0
fail=0
unknown=0
fail_cycles=()

echo "Config: $CYCLES cycles, stop/start delay ${DELAY}s, uptime per run ${UPTIME}s, service $SERVICE"
echo ""

POLL_INTERVAL=3

for i in $(seq 1 "$CYCLES"); do
  echo "=== cycle $i of $CYCLES ==="
  sudo systemctl stop "$SERVICE"
  sleep "$DELAY"
  start_ts=$(date +%s)
  sudo systemctl start "$SERVICE"

  # Poll the journal for a verdict so successful cycles complete in
  # a few seconds instead of waiting the full UPTIME bound.
  outcome=""
  elapsed=0
  while [ "$elapsed" -lt "$UPTIME" ]; do
    sleep "$POLL_INTERVAL"
    elapsed=$((elapsed + POLL_INTERVAL))
    window=$(journalctl -u "$SERVICE" --since="@$start_ts" --no-pager 2>/dev/null)
    if echo "$window" | grep -q "p2os setup failed"; then
      outcome="fail"
      break
    elif echo "$window" | grep -qE "Connected to .*Pioneer"; then
      outcome="pass"
      break
    fi
  done

  case "$outcome" in
    pass)
      echo "  PASS (${elapsed}s)"
      pass=$((pass + 1))
      ;;
    fail)
      echo "  FAIL — p2os setup failed (${elapsed}s)"
      fail=$((fail + 1))
      fail_cycles+=("$i")
      ;;
    *)
      echo "  UNKNOWN — no Connected/failed marker in ${UPTIME}s window"
      unknown=$((unknown + 1))
      fail_cycles+=("$i")
      ;;
  esac
done

echo ""
echo "=== summary ==="
echo "pass:    $pass / $CYCLES"
echo "fail:    $fail / $CYCLES"
echo "unknown: $unknown / $CYCLES"
if [ ${#fail_cycles[@]} -gt 0 ]; then
  echo "failed cycles: ${fail_cycles[*]}"
  exit 1
fi
exit 0
