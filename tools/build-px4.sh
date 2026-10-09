#!/usr/bin/env bash
# Build PX4 SITL (x500_depth) and the Micro-XRCE-DDS agent from the prepared
# pinned sources, staging the results under .tmp/simulation/px4 for the
# simulation image build:
#
#   px4/Tools/simulation/gz/{models,worlds}   PX4 gazebo models and worlds
#   px4/build/px4_sitl_default/{bin,etc}      built px4 binary and startup scripts
#   px4/build/px4_sitl_default/src/modules/simulation/gz_plugins
#   px4/xrce-agent/                           MicroXRCEAgent install tree
#
# Both builds run inside the simulation build image so the binaries link against
# the same Gazebo version the runtime uses. The staged tree is consumed by
# docker/compose.sim.yaml as the `px4_builder` additional context; it is a
# task-specific build product under .tmp and can be deleted after the image
# is built.
#
# Usage: bash tools/build-px4.sh [destination]
#   destination default: .tmp/simulation/px4
#   sources     default: .tmp/simulation/px4-src (from tools/prepare-px4.sh)
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source_dir=${PX4_SOURCE_DIR:-$root/.tmp/simulation/px4-src}
destination=${1:-$root/.tmp/simulation/px4}

if [ ! -d "$source_dir/px4_autopilot" ] || [ ! -d "$source_dir/micro_xrce_dds_agent" ]; then
    echo "PX4 sources not found under $source_dir; run tools/prepare-px4.sh first" >&2
    exit 1
fi
if [ -e "$destination" ]; then
    echo "Destination exists; remove it or choose a new directory: $destination" >&2
    exit 1
fi

image=${SIM_BUILD_IMAGE:-greenhouse-sim-build:local}
build_jobs=${BUILD_JOBS:-2}
if ! [[ "$build_jobs" =~ ^[1-9][0-9]*$ ]]; then
    echo "BUILD_JOBS must be a positive integer" >&2
    exit 1
fi
if ! docker image inspect "$image" >/dev/null 2>&1; then
    echo "Build image $image not found; run docker compose -f docker/compose.sim.yaml build simulation-build first" >&2
    exit 1
fi

mkdir -p "$destination/build"

# PX4 SITL firmware. Stage only what the runtime needs: the px4 binary,
# the startup scripts (etc/) and the gz plugins; the 1.5 GB intermediate
# build tree stays out of the image.
docker run --rm \
    --network host -e HTTP_PROXY -e HTTPS_PROXY -e NO_PROXY \
    --memory "${SIM_BUILD_MEMORY:-4g}" --memory-swap "${SIM_BUILD_SWAP_LIMIT:-6g}" \
    --cpus "$build_jobs" \
    -e BUILD_JOBS="$build_jobs" \
    -v "$source_dir/px4_autopilot":/px4-src:ro \
    -v "$destination":/px4-out \
    -w /px4-src \
    "$image" \
    bash -lc '
set -eo pipefail
source /opt/ros/jazzy/setup.bash
# Build on the Linux filesystem. NTFS bind mounts assign ownership to newly
# cloned ExternalProject repositories and make Git reject their checkout.
# Keep the prepared source tree read-only and retain its pinned Git metadata.
mkdir -p /tmp/px4-work
tar -C /px4-src --exclude=./build -cf - . | tar -C /tmp/px4-work --no-same-owner -xf -
cd /tmp/px4-work
python3 -m venv --system-site-packages /tmp/px4-python
source /tmp/px4-python/bin/activate
pip install --no-cache-dir -r Tools/setup/requirements.txt
export GZ_DISTRO=harmonic
# The PX4 wrapper discovers -j through ps T, which fails without a TTY.
# Pass the make variable explicitly so its Ninja invocation is also bounded.
make -j"$BUILD_JOBS" j="$BUILD_JOBS" px4_sitl_default
mkdir -p /px4-out/build/px4_sitl_default
cp -r Tools /px4-out/Tools
cp -r build/px4_sitl_default/bin /px4-out/build/px4_sitl_default/bin
cp -r build/px4_sitl_default/etc /px4-out/build/px4_sitl_default/etc
mkdir -p /px4-out/build/px4_sitl_default/src/modules/simulation
cp -r build/px4_sitl_default/src/modules/simulation/gz_plugins \
      /px4-out/build/px4_sitl_default/src/modules/simulation/gz_plugins
git rev-parse HEAD > /px4-out/source-revisions.txt
git submodule status --recursive >> /px4-out/source-revisions.txt
'

# Micro-XRCE-DDS agent (v2.4.3, required by PX4 v1.17; v3.x is incompatible).
docker run --rm \
    --network host -e HTTP_PROXY -e HTTPS_PROXY -e NO_PROXY \
    --memory "${SIM_BUILD_MEMORY:-4g}" --memory-swap "${SIM_BUILD_SWAP_LIMIT:-6g}" \
    --cpus "$build_jobs" \
    -e BUILD_JOBS="$build_jobs" \
    -v "$source_dir/micro_xrce_dds_agent":/agent-src:ro \
    -v "$destination":/px4-out \
    "$image" \
    bash -lc '
set -eo pipefail
cd /agent-src
mkdir -p /tmp/agent-work
tar -C /agent-src --exclude=./build -cf - . | tar -C /tmp/agent-work --no-same-owner -xf -
cmake -S /tmp/agent-work -B /tmp/agent-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xrce-agent
cmake --build /tmp/agent-build --parallel "$BUILD_JOBS"
cmake --install /tmp/agent-build
cp -r /opt/xrce-agent /px4-out/xrce-agent
git -C /tmp/agent-work rev-parse HEAD >> /px4-out/source-revisions.txt
'
echo "PX4 SITL and XRCE agent staged in $destination"
