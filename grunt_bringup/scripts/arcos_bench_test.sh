#!/usr/bin/env bash
# ARCOS drive-fault bench test harness.
#
# Brings up p2os_driver standalone (tank-drive / VEL2 path enabled,
# sonar off) and a diagnostic bag recorder, then waits while you run
# the test by hand. It does NOT drive the robot — you publish wheel_cmd
# and hand-spin the wheels yourself.
#
# Preconditions:
#   - grunt.service stopped (this runs p2os standalone).
#   - Robot WHEELS OFF THE GROUND, free to spin.
#
# When finished press Enter (or Ctrl-C): the bag is closed cleanly and
# p2os is shut down. Then analyse with:
#   cd ~/ros2_ws/src/grunt/grunt_analysis
#   python3 -m grunt_analysis.arcos <bag_dir>

set -uo pipefail

PORT="/dev/grunt_p3at"
NS="/grunt1"
BAG_ROOT="$HOME/bag_files/grunt"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
BAG_DIR="$BAG_ROOT/arcos_bench_$STAMP"
P2OS_LOG="$BAG_ROOT/arcos_bench_${STAMP}_p2os.log"
REC_LOG="$BAG_ROOT/arcos_bench_${STAMP}_record.log"

TOPICS=(
  "$NS/motor_stall"     # per-wheel stall flags + left_vel/right_vel
  "$NS/wheel_cmd"       # VEL2 tank-drive command
  "$NS/cmd_vel"         # recorded too, in case teleop is used
  "$NS/cmd_vel_joy"
  "$NS/motor_state"     # ARCOS motor-enable bit
  "$NS/pose"            # p2os wheel odometry (averaged twist)
  "$NS/battery_state"
)

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
mkdir -p "$BAG_ROOT"

P2OS_PID=""
BAG_PID=""

cleanup() {
  echo
  echo "[bench] stopping..."
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null
    wait "$BAG_PID" 2>/dev/null
    echo "[bench] bag closed: $BAG_DIR"
  fi
  if [[ -n "$P2OS_PID" ]] && kill -0 "$P2OS_PID" 2>/dev/null; then
    kill -INT "$P2OS_PID" 2>/dev/null
    wait "$P2OS_PID" 2>/dev/null
    echo "[bench] p2os stopped"
  fi
  echo
  echo "[bench] analyse with:"
  echo "  cd ~/ros2_ws/src/grunt/grunt_analysis"
  echo "  python3 -m grunt_analysis.arcos $BAG_DIR"
}
trap 'exit 130' INT TERM
trap cleanup EXIT

# --- p2os, tank-drive path enabled --------------------------------------
echo "[bench] starting p2os_driver (enable_wheel_cmd=true, sonar off)..."
ros2 run p2os_driver p2os_driver --ros-args \
  -r __ns:="$NS" \
  -p port:="$PORT" \
  -p baud_rate:=115200 \
  -p use_sonar:=false \
  -p enable_wheel_cmd:=true \
  >"$P2OS_LOG" 2>&1 &
P2OS_PID=$!

# --- wait for the ARCOS link (motor_stall actually flowing) -------------
echo "[bench] waiting for ARCOS link ($NS/motor_stall)..."
for i in $(seq 1 30); do
  if timeout 3 ros2 topic echo "$NS/motor_stall" --once >/dev/null 2>&1; then
    echo "[bench] ARCOS link up."
    break
  fi
  if ! kill -0 "$P2OS_PID" 2>/dev/null; then
    echo "[bench] p2os exited during startup — see $P2OS_LOG" >&2
    exit 1
  fi
  if [[ "$i" -eq 30 ]]; then
    echo "[bench] timed out waiting for motor_stall — see $P2OS_LOG" >&2
    exit 1
  fi
  sleep 1
done

# --- diagnostic bag -----------------------------------------------------
echo "[bench] recording -> $BAG_DIR"
ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}" >"$REC_LOG" 2>&1 &
BAG_PID=$!
sleep 2

cat <<MSG

[bench] =============================================================
[bench]  p2os is up and the bag is recording. Run the test now,
[bench]  WHEELS OFF THE GROUND:
[bench]
[bench]   Phase A  motors DISABLED - hand-spin each wheel, watch
[bench]            $NS/motor_stall left_vel / right_vel
[bench]   Phase B  motors ENABLED  - drive one wheel at a time, then
[bench]            both equal, via $NS/wheel_cmd. Start by commanding
[bench]            the LEFT wheel only and confirm the LEFT wheel
[bench]            spins (validates the VEL2 byte order).
[bench]
[bench]  Per-wheel command (you run this — the script does not drive):
[bench]   ros2 topic pub -r 10 $NS/wheel_cmd p2os_msgs/msg/WheelCmd "{left: 0.1, right: 0.0}"
[bench]
[bench]  Press Enter when finished to close the bag cleanly.
[bench] =============================================================
MSG

read -r _
# cleanup() runs via the EXIT trap
