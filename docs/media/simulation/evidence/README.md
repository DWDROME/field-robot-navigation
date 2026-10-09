# Simulation evidence

The fixed matrix completed on 2026-10-07: **27 independent runs, 8 pass and 19 fail**, with no blocked or not-run entries. All 18 ground runs failed; maize and orchard each passed all three X500 runs, while pipeline passed two and failed one because takeoff left the checked home column. Failed navigation and flight records are retained and are not counted as success. This evidence does not claim successful ground navigation, correct dangerous-area refusal, three-dimensional avoidance, terrain following or real-hardware performance.

## Final runtime and media

- [matrix-validation.json](matrix-validation.json): expected run IDs, common image/implementation identity, 29 installed-source comparisons, resources, empty remaining-process lists, 15 reviewed 1920×1080 images with hashes, and ULog inventory.
- [delivery-integrity.json](delivery-integrity.json): 27 stored root records, CSV membership, measured-case fields, trace/ULog existence, 15 image hashes/status links, 29 current source hashes and 114 delivery links checked. Repeat read-only with `python3 docs/media/simulation/evidence/check-delivery.py` from the repository root; it validates this frozen dataset rather than rerunning simulation.
- [final-runtime-health.json](final-runtime-health.json): after all 27 runs and the RViz supplement, cgroup OOM and OOM kill are zero, no live Gazebo world remains, and the installed FAR binary matches the recorded SHA256. The task container was then stopped.
- [JSON](candidate-matrix/matrix_results.json) and [CSV](candidate-matrix/matrix_results.csv): nine combinations, three fixed independent resets each. Per-run JSON and directories retain configuration, localization source, thresholds, startup logs, task logs, actual contacts/attitude/timing, traces and PX4 ULogs.
- [matrix-execution.log](matrix-execution.log): checkpointed matrix execution. The runner exits nonzero because measured task failures exist; that is not an infrastructure success claim.
- [capture supplement](capture-supplement/pipeline_jackal_j100_rviz_retry.json): identical-image and identical-implementation independent pipeline run used only to obtain the missing RViz image. Its navigation still failed and it is excluded from the 27-run statistics. The original first-round RViz capture failure remains in the matrix.
- [album](../README.md): 3 panoramas, 9 task images and 3 RViz images, personally inspected for scene/robot/path visibility. All have PNG/JPG and linked JSON metadata. RViz retains the first real FAR path for display only; a visible historical path is not proof that control succeeded.
- [final-ground-command-ownership.json](final-ground-command-ownership.json): actual four-stage TwistStamped command types, base_link frames, current stamps and unique publisher counts during the final candidate. All observed commands were zero. One graph node name was `_NODE_NAME_UNKNOWN_` during teardown and is retained as observed.

Container paths `/evidence/candidate-matrix` and `/evidence/capture-supplement` map to the corresponding directories here. Image-owned dangling `etc` symlinks are omitted from the host copy; measured parameters, traces, logs and ULogs remain. Ground localization is simulated-sensor FAST-LIO; no ground-truth-assisted run is counted in this matrix.

## Source, assets and builds

- [build-provenance.json](build-provenance.json), [installed-source-and-geometry.json](installed-source-and-geometry.json): final image `sha256:dea19f601f3e14ace38fd3da54b20d3b1104ed70fc37b4e29be4e7f3caa9c09d`, current SIM source hashes and actual three-site route/home/collision geometry.
- [maize-reproducibility.json](maize-reproducibility.json): two independent generations matched six artifacts with seed 20261006, 1015 public plants, no weeds/litter and recorded actual generator arguments.
- [px4-source-revisions.txt](px4-source-revisions.txt), [px4-bootstrap-build.log](px4-bootstrap-build.log): pinned firmware/models/Agent and successful fresh PX4/Agent build. A leading `-` marks an uninitialized unused firmware submodule; build success alone does not prove flight.
- [far-free-terrain-build.log](far-free-terrain-build.log): cached isolated FAR build after patch 0005, 34.7 seconds, 29 verified build inputs. Final executable SHA256 is `d42fe736b103ff5c82c8b684e0f83c08cbcbc81052f38d6601de9424c2d06191`. The production/SIM build chain succeeded earlier; the whole production build was not repeated after patch 0005. The final candidate uses that matching incremental FAR binary plus current SIM layers.
- [change-files.md](change-files.md): added/modified task files, their purpose and actual temporary duplicate-log deletions. Unrelated uncommitted baseline changes are excluded.

## Focused integration regressions

- [serialization-regression.log](serialization-regression.log), [final-source-regression.log](final-source-regression.log): three generated ROS command/finite-LiDAR-field regressions on the recorded preceding image; relevant air/filter sources are unchanged. These are software interface tests, not navigation or flight proof.
- [process-title-cleanup-regression.log](process-title-cleanup-regression.log): two real reparented-child tests passed in 7.13 seconds, covering ordinary argv and Gazebo combined process titles; an unrelated similarly named sibling survives. [Diagnosis](process-cleanup-diagnosis.md) identifies the escaped Gazebo server that caused an earlier excluded matrix OOM.
- [px4-live-telemetry-startup-regression.log](px4-live-telemetry-startup-regression.log): one synthetic cross-process DDS test passed in 1.69 seconds on isolated domain 179. A discovered topic without packets does not satisfy startup; a real finite VehicleLocalPosition packet does. It is not counted as flight evidence.
- [orchard shared bridge](orchard-shared-bridge-diagnostic.json), [clock](orchard-original-clock-diagnostic.json), [isolated bridge](orchard-isolated-bridge-diagnostic.json): actual camera contention isolated from fresh raw Gazebo sensor timing. Independent IMU/LiDAR/camera processes keep demanded RGB/depth traffic from starving IMU; original data, rates and safety freshness limits are preserved.

## Diagnostic limitations

- [ground-nearfield-diagnostic.json](ground-nearfield-diagnostic.json): older matching-control diagnostic observed all six near-body cells unknown and 96 of 100 cells unknown in a 2 m square. MPPI rejected trajectories; no unknown-as-free substitution or footprint/threshold relaxation was applied. Final matrix ground failures and zero commands are retained separately above.
- [pipeline-air-spawn-diagnosis.md](pipeline-air-spawn-diagnosis.md), [original spawn pose](pipeline-original-spawn-pose.json), [staging ground](pipeline-staging-ground.json), [flat-spawn QA](flat-spawn-air-regression/pipeline_x500_flat_spawn_qa.json): physical pitch failure on uneven terrain, actual collision-box/flat-patch analysis and a full-flight diagnostic after relocating the home site without modifying terrain.
- [diagnostics/inventory.json](diagnostics/inventory.json): inventory of 823 excluded diagnostic files, 238250984 bytes, copied and individually hash-matched before cache cleanup. The old interrupted/failed matrices, QA logs/traces/ULogs and diagnostic media remain in the local workspace; the initial GitHub snapshot includes this inventory but omits those bulk historical files. The final candidate matrix and capture supplement above retain their complete measured records. Historical diagnostics must not be mixed with the frozen final candidate matrix. Static slopes and clearance are geometry measurements, not tested vehicle traversability, flexible-crop interaction, sinking or water physics.

Comet Native independent acceptance has not yet run. This index separates implementation/interface/build evidence from measured task outcomes and does not replace that acceptance.

[Builder preparation review](acceptance-self-review.json) covers A1–A17. A17 is explicitly incomplete because required temporary cleanup remains pending; no complete candidate has been submitted. Ground autonomy remains unproved even though the assembly and failed benchmark attempts are implemented.

[Cache cleanup scope and recovery](cache-cleanup.md) records the exact isolated task cache, preserved evidence and the automatic approval rejection. Required temporary cleanup remains pending; no source or main-workspace deletion is proposed.
