#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/greenhouse_path_follower/setup.bash
set -u

ros2 run cmu_path_follower_generic path_follower_generic \
  --ros-args \
  -p autonomy_mode:=true \
  > /tmp/path-follower-graph-smoke.log 2>&1 &
follower_pid=$!

cleanup() {
  kill -TERM "${follower_pid}" 2>/dev/null || true
  wait "${follower_pid}" 2>/dev/null || true
  cat /tmp/path-follower-graph-smoke.log
}
trap cleanup EXIT

sleep 1
python3 /tmp/run_graph_smoke.py
