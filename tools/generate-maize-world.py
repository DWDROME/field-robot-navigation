#!/usr/bin/env python3
"""Pre-generate the virtual maize field world with a fixed configuration and seed.

Runs at image build time so that the world is reproducible and self-contained:
no runtime downloads, no personal cache directories. The generator writes into
$ROS_HOME/virtual_maize_field; this script then copies the generated world,
heightmap and ground-truth files into the destination directory so the runtime
only needs GZ_SIM_RESOURCE_PATH.

The configuration deliberately uses only openly licensed models (maize, ground,
start marker): weeds, litter and location markers stay disabled, and the
unpublished dandelion models are never referenced.
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

# Fixed generation parameters for the simulation matrix. 12 rows with mixed
# straight and curved segments reproduce the documented field layout; the
# per-row hole_prob/hole_size_max lists create the missing-plant pattern;
# headland, ditch and elevation noise shape the terrain. seed fixes the
# random field layout.
#
# The upstream `sincurved` segment generator does not terminate with these
# row/hole parameters (observed: >40 min CPU without output); the
# straight/curved mix provides both row shapes without that defect.
GENERATOR_ARGS = [
    "--row_length", "10",
    "--rows_count", "12",
    "--row_segments", "straight", "curved",
    "--hole_prob", "0.06", "0.06", "0.06", "0.05", "0.03", "0.02", "0.02", "0.01", "0.0", "0.0", "0.0", "0.0",
    "--hole_size_max", "10", "10", "9", "7", "4", "5", "5", "3", "2", "0", "0", "0",
    "--ground_elevation_max", "0.2",
    "--ground_headland", "2.0",
    "--ground_ditch_depth", "0.3",
    "--litters", "0",
    "--weeds", "0",
    "--seed", "20261006",
]
# Note: boolean flags (ghost_objects, location_markers) are omitted because the
# upstream parser builds them with type=bool, so any string argument parses as
# True; the function defaults (False) are the intended values.


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path,
                        help="Directory that receives generated.world, heightmap and ground truth")
    parser.add_argument("--seed", type=int, default=None,
                        help="Override the fixed seed (default: 20261006)")
    args = parser.parse_args()

    destination = args.destination.resolve()
    if destination.exists() and any(destination.iterdir()):
        print(f"Destination already populated: {destination}", file=sys.stderr)
        return 1
    destination.mkdir(parents=True, exist_ok=True)

    generator_args = list(GENERATOR_ARGS)
    if args.seed is not None:
        seed_index = generator_args.index("--seed") + 1
        generator_args[seed_index] = str(args.seed)

    subprocess.run(
        ["ros2", "run", "virtual_maize_field", "generate_world", *generator_args],
        check=True,
    )

    ros_home = Path(subprocess.check_output(
        ["bash", "-c", 'source /opt/ros/jazzy/setup.bash && echo "${ROS_HOME:-$HOME/.ros}"'],
        text=True).strip())
    cache = ros_home / "virtual_maize_field"
    for name in ("generated.world", "virtual_maize_field_heightmap.png",
                 "gt_map.csv", "markers.csv", "driving_pattern.txt",
                 "robot_spawner.launch.py"):
        source = cache / name
        if not source.is_file():
            print(f"Missing generated artifact: {source}", file=sys.stderr)
            return 1
        shutil.copy2(source, destination / name)

    # The world template embeds the generation-time cache path for the
    # heightmap; rewrite it to the delivery location so the runtime never
    # depends on the build-time cache directory.
    world_file = destination / "generated.world"
    world_file.write_text(world_file.read_text().replace(
        str(cache), str(destination)))

    # The world template references model:// URIs resolved from the installed
    # package share; record the resource roots for the runtime environment.
    from ament_index_python.packages import get_package_share_directory
    (destination / "resource_paths.txt").write_text(
        str(Path(get_package_share_directory('virtual_maize_field'))/'models')+'\n')
    (destination/'generation.json').write_text(json.dumps({
        'seed':int(generator_args[generator_args.index('--seed')+1]),
        'command':['ros2','run','virtual_maize_field','generate_world',*generator_args],
        'preset':None,
        'preset_behavior':'Upstream config_file branch loads YAML and ignores CLI overrides; no preset is supplied here',
        'models':['maize_01','maize_02'],'weeds':0,'litters':0,
        'artifact_sha256':{name:hashlib.sha256((destination/name).read_bytes()).hexdigest()
            for name in ['virtual_maize_field_heightmap.png','gt_map.csv','markers.csv','driving_pattern.txt']}
    },indent=2)+'\n')
    print(f"Generated maize field world in {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
