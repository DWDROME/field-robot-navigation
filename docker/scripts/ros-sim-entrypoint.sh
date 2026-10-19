#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
for simulation_prefix in /opt/greenhouse_far /opt/greenhouse /opt/maize_field /opt/greenhouse_air; do
    source "${simulation_prefix}/local_setup.bash"
done
export PYTHONPATH=/opt/sim-python/lib/python3.12/site-packages:${PYTHONPATH:-}
exec "$@"
