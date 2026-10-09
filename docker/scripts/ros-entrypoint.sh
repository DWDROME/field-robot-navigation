#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
# Load the external packages before the maintained robot workspace.
for greenhouse_prefix in /opt/greenhouse_far /opt/greenhouse; do
  source "${greenhouse_prefix}/local_setup.bash"
done
exec "$@"
