#!/usr/bin/env bash
# Build a driver-only workspace without modifying the source manifest.
set -eo pipefail
source_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
edition=${1:-}
case "$edition" in
  ROS1) manifest=package_ROS1.xml ;;
  ROS2|humble|jazzy) manifest=package_ROS2.xml ;;
  *) echo 'Usage: bash build.sh ROS1|ROS2|humble|jazzy [new-workspace]' >&2; exit 2 ;;
esac
if [[ -n ${2:-} ]]; then
  workspace=$(realpath -m "$2")
  if [[ -e "$workspace" ]]; then
    echo "Workspace already exists; use a new directory: $workspace" >&2
    exit 2
  fi
  mkdir -p "$workspace"
else
  project_root=$(cd "$source_dir/../../.." && pwd)
  mkdir -p "$project_root/.tmp"
  workspace=$(mktemp -d "$project_root/.tmp/livox-build-XXXXXX")
fi
mkdir -p "$workspace/src/livox_ros_driver2"
for entry in "$source_dir"/*; do
  [[ $(basename "$entry") == package.xml ]] && continue
  ln -s "$entry" "$workspace/src/livox_ros_driver2/$(basename "$entry")"
done
cp "$source_dir/$manifest" "$workspace/src/livox_ros_driver2/package.xml"
printf 'Build workspace: %s\n' "$workspace"
cd "$workspace"
if [[ $edition == ROS1 ]]; then
  catkin_make -j"${BUILD_JOBS:-2}" -DROS_EDITION=ROS1
else
  colcon build --packages-select livox_ros_driver2 --cmake-args \
    -DROS_EDITION=ROS2 -DDISTRO_ROS="${ROS_DISTRO:-${edition/ROS2/}}"
fi
