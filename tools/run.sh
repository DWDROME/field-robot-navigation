#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: bash tools/run.sh <mapping|localization|navigation> hardware [map:=/absolute/map.pcd] [launch arguments...]"
}

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  usage
  exit 0
fi
mode=${1:-navigation}
environment=${2:-hardware}
if (( $# > 0 )); then shift; fi
if (( $# > 0 )); then shift; fi
case "$mode" in
  mapping|localization|navigation) ;;
  *) usage >&2; exit 2 ;;
esac
if [[ "$environment" == simulation ]]; then
  echo "Gazebo Harmonic belongs to upgrade/ros2. Switch branches to use its simulation environment; ROS 1 simulation is not adapted on main." >&2
  exit 2
fi
if [[ "$environment" != hardware ]]; then
  usage >&2
  exit 2
fi
if [[ "$mode" != mapping ]]; then
  map_provided=false
  for argument in "$@"; do
    if [[ "$argument" == map:=?* ]]; then map_provided=true; fi
  done
  if [[ "$map_provided" != true ]]; then
    echo "Provide the existing 3D map with map:=/absolute/map.pcd." >&2
    exit 2
  fi
fi
exec roslaunch rover_launch system.launch "mode:=$mode" "$@"
