#!/usr/bin/env bash
# Build the maintained FAST-LIVO2 package and its pinned Noetic dependencies.
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(realpath -m "${1:?usage: build-fastlivo2.sh workspace [check]}")
source /opt/ros/noetic/setup.bash
if [[ ! -d "$workspace/deps" ]]; then
  cache_args=()
  [[ -z ${DEPENDENCY_CACHE:-} ]] || cache_args=(--cache "$DEPENDENCY_CACHE")
  python3 "$root/tools/prepare-dependencies.py" ros1 "$workspace/deps" sophus rpg_vikit "${cache_args[@]}"
fi
mkdir -p "$workspace/src"
for pair in "fast_livo:$root/src/localization/fast_livo" "rpg_vikit:$workspace/deps/rpg_vikit"; do
  name=${pair%%:*}; path=${pair#*:}
  if [[ -e "$workspace/src/$name" || -L "$workspace/src/$name" ]]; then
    [[ $(realpath "$workspace/src/$name") == "$path" ]] || { echo "Conflicting package: $workspace/src/$name" >&2; exit 2; }
  else
    ln -s "$path" "$workspace/src/$name"
  fi
done
cmake -S "$workspace/deps/sophus" -B "$workspace/sophus-build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$workspace/sophus-build" --target Sophus -j"${BUILD_JOBS:-2}"
# Preserve upstream output paths without shipping historical logs or datasets.
mkdir -p "$root/src/localization/fast_livo/Log/"{pcd,image,result,Colmap/sparse/0}
cd "$workspace"
catkin_make -DFASTLIVO_MEMORY_SANITIZE="${FASTLIVO_MEMORY_SANITIZE:-OFF}" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DSophus_DIR="$workspace/sophus-build" \
  -DCATKIN_WHITELIST_PACKAGES='vikit_common;vikit_ros;fast_livo' \
  -j"${BUILD_JOBS:-2}" -l2 --make-args fastlivo_mapping memory_lifecycle
if [[ ${2:-} == check ]]; then
  export ROS_MASTER_URI=http://127.0.0.1:11381
  export ROS_HOME="$workspace/test-ros-home"
  mkdir -p "$ROS_HOME"
  roscore -p 11381 > "$ROS_HOME/master.log" 2>&1 & master=$!
  trap 'kill "$master" 2>/dev/null || true; wait "$master" 2>/dev/null || true' EXIT
  for attempt in {1..50}; do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep 0.1
  done
  rosparam list >/dev/null
  test_command=("$workspace/devel/lib/fast_livo/memory_lifecycle")
  if [[ ${FASTLIVO_MEMORY_SANITIZE:-OFF} == ON ]]; then
    test_command=(setarch "$(uname -m)" -R "${test_command[@]}")
  fi
  LD_BIND_NOW=1 ASAN_OPTIONS=handle_segv=0:fast_unwind_on_malloc=0:malloc_context_size=40 \
    LSAN_OPTIONS="suppressions=$root/src/localization/fast_livo/test/lsan-noetic.supp:print_suppressions=1" \
    timeout 60s "${test_command[@]}"
fi
