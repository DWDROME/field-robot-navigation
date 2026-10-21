#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
exec catkin_make -C "$project_dir" -j"${BUILD_JOBS:-2}" -DROS_EDITION=ROS1 "$@"
