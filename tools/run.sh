#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: bash tools/run.sh <mapping|localization|navigation> <hardware|simulation> [launch arguments...]"
}

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  usage
  exit 0
fi
mode=${1:-navigation}
environment=${2:-hardware}
if (( $# > 0 )); then shift; fi
if (( $# > 0 )); then shift; fi

case "$mode/$environment" in
  navigation/hardware)
    exec ros2 launch greenhouse_nav2_bringup closed_loop.launch.py "$@"
    ;;
  navigation/simulation)
    exec ros2 launch greenhouse_nav2_bringup simulation_matrix.launch.py "$@"
    ;;
  mapping/simulation)
    exec ros2 launch greenhouse_nav2_bringup bringup.launch.py \
      profiles:=mapping-fastlio host_profile:=simulation "$@"
    ;;
  mapping/hardware)
    echo "The generic Jazzy mapping profile currently supports simulation/dev-ci. Hardware mapping needs its calibrated bringup configuration; see docs/build.md." >&2
    exit 2
    ;;
  localization/hardware|localization/simulation)
    echo "PCD localization is not connected in upgrade/ros2. Use the ROS 1 localization mode on main." >&2
    exit 2
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
