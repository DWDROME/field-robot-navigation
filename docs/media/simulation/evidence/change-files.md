# Simulation change file inventory

This inventory covers `gazebo-sim-matrix`, including work completed before crash recovery. It excludes the unrelated source relocation and other uncommitted baseline work copied from the main workspace. No source file was deleted, staged, committed or pushed by this continuation.

## Existing integration files updated

- `README.md`, `docs/simulation.md`, `third_party/README.md`: actual scope, build/runtime commands, source attribution, result and image documentation.
- `docker/Dockerfile.sim`, `docker/compose.sim.yaml`, `docker/.dockerignore`, `docker/config/simulation.packages`: isolated SIM build stages, pinned dependencies, runtime resources, assets and installation.
- `tools/prepare-simulation.sh`, `tools/prepare-px4.sh`, `tools/build-px4.sh`, `tools/generate-maize-world.py`: pinned source preparation, independent PX4/Agent bootstrap and deterministic field generation.
- `third_party/ros2.repos`, `third_party/px4.repos`: exact external simulation revisions.
- `src/bringup/greenhouse_nav2_bringup/CMakeLists.txt`: installation and relevant regression test registration.
- `src/navigation/greenhouse_mppi_navigation/launch/closed_loop.launch.py`: dense observed simulation returns for the existing navigation assembly.

## New runtime and validation files

- `docker/compose.sim.runtime.yaml`: necessary runtime overlay preventing unintended production rebuilds.
- `docker/compose.sim.wsl.yaml`, `docker/scripts/ros-sim-entrypoint.sh`: WSL GPU mapping and isolated ROS/FAR/air prefixes.
- `tools/inspect-simulation-geometry.py`: reproducible full asset, route, home-column and actual X500 collision checks.
- `third_party/patches/autonomy_stack_go2/0004-use-current-odometry-for-goal-reevaluation.patch`: fixes the reproduced first-goal null-odometry crash.
- `third_party/patches/autonomy_stack_go2/0005-initialize-with-observed-free-terrain.patch`: permits graph initialization from actually observed free terrain without requiring an obstacle.

Under `src/bringup/greenhouse_nav2_bringup/`:

- `launch/simulation_matrix.launch.py`, `launch/simulation_ground_vehicle.launch.py`, `launch/simulation_x500.launch.py`: one world/clock entry, physical ground vehicle and attached PX4 assembly.
- `scripts/run_matrix.py`, `scripts/run_simulation_run.py`: independent fixed runs, metrics, checkpoints, source identity, real PX4 startup telemetry and owned-process cleanup.
- `scripts/simulation_ground_startup.py`, `scripts/simulation_lidar_filter.py`: measured physical settling and finite-return filtering with original valid data preserved.
- `scripts/capture_simulation_image.py`, `scripts/capture_simulation_rviz.py`: real Gazebo and RViz PNG/JPG capture with linked metadata.
- `config/simulation/robots/jackal_j100.yaml`, `config/simulation/robots/husky_a200.yaml`: distinct generated robot assets and limits.
- `config/simulation/sensor-bridge-j100.yaml`, `config/simulation/sensor-bridge-a200.yaml`: per-platform actual sensor topic mappings.
- `config/simulation/deployment-jackal_j100.yaml`, `config/simulation/deployment-husky_a200.yaml`, `config/simulation/command-gate.yaml`: project navigation, unique gate and platform sink configuration.
- `config/simulation/tasks/ground_sites.yaml`: normal, terrain and danger cases tied to measured geometry.
- `config/simulation/dds-loopback.xml`, `config/simulation/navigation.rviz`, `worlds/simulation_camera.sdf`: observed DDS transport requirement, actual navigation display and fixed 1920×1080 capture camera.
- `test/test_simulation_lidar_filter.py`, `test/test_simulation_process_cleanup.py`, `test/test_simulation_px4_startup.py`: field preservation, escaped-process cleanup and discovered-topic-without-message regressions.

Under the new `src/bringup/greenhouse_sim_air/` package:

- `package.xml`, `setup.py`, `setup.cfg`, `resource/greenhouse_sim_air`, `greenhouse_sim_air/__init__.py`: necessary isolated Python package metadata.
- `greenhouse_sim_air/air_mission.py`, `greenhouse_sim_air/micro_xrce_agent.py`, `greenhouse_sim_air/simulation_gcs.py`: real PX4 Offboard state machine, pinned DDS Agent and local GCS heartbeat.
- `config/x500_sites.yaml`, `test/test_air_commands.py`: fixed checked site missions and generated ROS float serialization regression.

## Generated evidence and temporary cleanup

`docs/media/simulation/` contains 15 final real PNG/JPG pairs, JSON sidecars and the album. Its `evidence/` directory preserves build/source/geometry proofs, diagnosis, regression logs, final matrix records/traces/ULogs and an excluded same-version RViz supplement. `diagnostics/` preserves 823 older diagnostic files with a hash inventory plus ten copied host-side checks; those do not change final statistics. `check-delivery.py` supplies a portable read-only check of the frozen data, source hashes and delivery links. `acceptance-self-review.json` records preparation evidence and remaining work, not an independent verdict; `cache-cleanup.md` makes the remaining cache removal concrete and reviewable.

Four temporary duplicate logs were deleted only after SHA256 equality with the preserved evidence copy was confirmed: `.tmp/simulation/px4-bootstrap-verified.log`, `.tmp/far-free-terrain-build-success.log`, `.tmp/simulation/bridge-isolation-overlay-build.log` and `.tmp/simulation/flat-air-spawn-overlay-build.log`. Automatic approval rejected recursive obsolete-build cleanup and a later batch of duplicate host-side files with `blocked by policy`; those commands did not run. All remaining task-local cache data is under this worktree's `.tmp`, now approximately 2.9 GiB, and required manual cleanup is pending after full evidence preservation. No source, main-workspace or other-change logs were removed. The measured container was stopped after all runs and final health capture.
