#!/usr/bin/env bash
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
source "$project_dir/devel/setup.bash"
exec bash "$project_dir/tools/run.sh" navigation hardware "$@"
