#!/usr/bin/env bash
# Build the ROS 1 SLAM packages in a fresh isolated catkin workspace.
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/noetic/setup.bash
if [[ -n ${1:-} ]]; then
  workspace=$(realpath -m "$1")
  shift
  if [[ -e "$workspace" ]]; then
    echo "Workspace already exists; use a new directory: $workspace" >&2
    exit 2
  fi
  mkdir -p "$workspace"
else
  mkdir -p "$root/.tmp"
  workspace=$(mktemp -d "$root/.tmp/ros1-slam-XXXXXX")
fi
mkdir -p "$workspace/src"
ln -s "$root/src/drivers/livox_ros_driver2" "$workspace/src/livox_ros_driver2"
ln -s "$root/src/localization/fast_lio_super" "$workspace/src/fast_lio_super"
ln -s "$root/src/localization/slam_octvox" "$workspace/src/slam_octvox"
printf 'Build workspace: %s\n' "$workspace"
cd "$workspace"
catkin_make -j"${BUILD_JOBS:-2}" -DROS_EDITION=ROS1 \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
