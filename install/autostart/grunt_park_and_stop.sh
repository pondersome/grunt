#!/usr/bin/env bash
# Pre-SIGTERM helper for the grunt systemd service.
#
# Called by grunt.service ExecStop before systemd cgroup-SIGTERMs the
# stack. Parks the arm to the bumper preset (so it's folded before any
# power cut) and publishes zero velocity briefly (so the chassis stops
# under active motor control rather than mid-motion).
#
# Time-bounded: if the stack is unhealthy and topics aren't live, the
# script still returns in a few seconds. systemd then proceeds with
# the SIGTERM cascade. The real grade-park story is a BT-level job
# (see memory/project_grade_park_backlog.md) — this helper is the
# best the systemd layer can do.

# Deliberately no `set -u`: ROS 2's setup.bash references uninitialized
# variables (COLCON_CURRENT_PREFIX and friends) as part of normal
# operation, so enabling -u makes sourcing fail with exit status 1
# before we can do anything useful — and that failure makes systemd's
# ExecStop report failure, which interferes with the SIGTERM cascade.
source /opt/ros/jazzy/setup.bash
source /home/karim/ros2_ws/install/setup.bash

PREFIX="${1:-grunt1}"

# Arm to bumper. -w 1 waits for a subscriber; -t 3 publishes three
# times for packet-loss tolerance. timeout caps total time at ~3s.
timeout 3 ros2 topic pub -w 1 -t 3 \
  "/${PREFIX}/arm_preset" std_msgs/msg/String \
  "{data: 'bumper'}" >/dev/null 2>&1 || true

# Brief zero-velocity hold on the nav source: continuous publish for
# ~1.5s so twist_mux sees a fresh source and p2os_driver's 200ms
# cmd_vel watchdog doesn't starve. If the operator is holding
# deadman, joystick (priority 100) still overrides.
timeout 2 ros2 topic pub -r 20 \
  "/${PREFIX}/cmd_vel_nav" geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  >/dev/null 2>&1 || true

# Small settle so the arm has time to reach bumper before serial closes.
sleep 0.5

exit 0
