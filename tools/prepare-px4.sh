#!/usr/bin/env bash
# Fetch the pinned PX4 SITL air-simulation dependencies into a new directory.
#
# PX4-Autopilot carries 29 git submodules, most of which are only needed for
# hardware targets. This script checks out the pinned commits, then initializes
# only the submodules that the SITL gz simulation path actually uses:
#   - src/modules/mavlink/mavlink  (MAVLink message definitions, px4 build dep)
#   - Tools/simulation/gz          (PX4-gazebo-models; the parent commit pins
#                                   the model submodule revision)
#   - Micro-XRCE-DDS-Client, libevents and GPS devices (SITL build dependencies)
# Micro-XRCE-DDS-Agent is fetched at the v2.4.3 revision that PX4 v1.17
# documents as the compatible DDS agent (v3.x is explicitly incompatible).
#
# The destination must not exist yet. Existing working trees are never reset.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
destination=${1:?usage: prepare-px4.sh <destination> [--cache <dir>]}
cache=""
if [ "${2:-}" = "--cache" ]; then
    cache=${3:?--cache requires a directory}
fi

if [ -e "$destination" ]; then
    echo "Destination exists; use the existing sources or choose a new directory: $destination" >&2
    exit 1
fi

mkdir -p "$destination"
destination=$(cd "$destination" && pwd)

fetch() {
    local name=$1 url=$2 version=$3
    local target=$destination/$name
    local origin=$url
    if [ -n "$cache" ] && [ -d "$cache/$name" ]; then
        origin=$cache/$name
    fi
    git -c core.autocrlf=false clone --no-checkout "$origin" "$target"
    git -C "$target" config core.autocrlf false
    git -C "$target" remote set-url origin "$url"
    git -C "$target" checkout --detach "$version"
    git -C "$target" diff --exit-code HEAD --
}

fetch px4_autopilot https://github.com/PX4/PX4-Autopilot.git \
    d6f12ad1c4f70ad3230afd7d86e971421e02fef4
git -C "$destination/px4_autopilot" submodule update --init --recursive --depth 1 \
    src/modules/mavlink/mavlink Tools/simulation/gz \
    src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client \
    src/lib/events/libevents src/drivers/gps/devices

fetch micro_xrce_dds_agent https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    73622810d984349b80bbac0ef55fc0b694d62222

echo "Prepared PX4 SITL dependencies in $destination"
