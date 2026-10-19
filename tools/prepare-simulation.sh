#!/usr/bin/env bash
# Stage the simulation image build contexts under .tmp/simulation/:
#
#   assets/maize_field_src   virtual_maize_field source (ros2-gz, pinned)
#   assets/px4_msgs          px4_msgs release/1.17 (pinned)
#   px4-src/px4_autopilot    PX4-Autopilot v1.17.0 with SITL submodules
#   px4-src/micro_xrce_dds_agent  Micro-XRCE-DDS-Agent v2.4.3
#   px4/                     staged PX4 SITL build (tools/build-px4.sh output)
#
# Sources come from tools/prepare-dependencies.py (ros2 manifests) and
# tools/prepare-px4.sh. Everything lands under .tmp (task-local, deletable
# after the image is built).
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
sim=$root/.tmp/simulation
mkdir -p "$sim/assets"

if [ ! -d "$sim/assets/maize_field_src" ]; then
    python3 "$root/tools/prepare-dependencies.py" ros2 "$sim/deps" virtual_maize_field px4_msgs
    cp -r "$sim/deps/virtual_maize_field" "$sim/assets/maize_field_src"
    cp -r "$sim/deps/px4_msgs" "$sim/assets/px4_msgs"
fi

if [ ! -d "$sim/px4-src/px4_autopilot" ]; then
    bash "$root/tools/prepare-px4.sh" "$sim/px4-src"
fi

echo "Simulation assets staged under $sim"
echo "Next: docker compose -f docker/compose.sim.yaml build simulation-build"
echo "Then: bash tools/build-px4.sh; docker compose -f docker/compose.sim.yaml build simulation"
