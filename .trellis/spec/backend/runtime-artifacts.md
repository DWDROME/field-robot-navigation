# Runtime Artifact Contracts

> File outputs created by mapping, graph tooling, and bag workflows.

---

## Classification Rule

Do not classify an asset from its extension alone.

| Class | Examples | Repository treatment |
|-------|----------|----------------------|
| Source configuration | YAML, JSON, launch, RViz | Version and review |
| Static algorithm input | local-planner PLY/TXT, FAR baseline data | Version and protect from broad ignores |
| Runtime-generated output | PCD scans, logs, timing CSV, recorded bags | Keep out of source history unless deliberately promoted |
| Deployment asset | map PCD, udev rule, device config | Track provenance and validate on target |

## Known Writers

### FAST-LIO family

Baseline FAST-LIO can write under its compiled source `ROOT_DIR`:

- `PCD/scans.pcd` on shutdown;
- `Log/pos_log.txt`;
- `Log/mat_pre.txt`;
- `Log/mat_out.txt`;
- `Log/dbg.txt`;
- optional `Log/fast_lio_time_log.csv`.

The localization and SUPER variants retain the same package-local
`ROOT_DIR` pattern. PCD saving can affect real-time performance and should
remain disabled in the greenhouse navigation entry unless a mapping/data
capture task explicitly enables it.

### FAR graph tools

- `boundary_handler` writes a graph file such as
  `src/far_planner/data/boundary_graph.vgh`.
- `graph_decoder` accepts `/save_file_dir` and uses the message value as the
  output path.
- These writers open output files in overwrite mode. A versioned VGH baseline
  can therefore be replaced by a runtime command.

### Rosbag

Livox ROS1 launch files can optionally run `rosbag record -a`. The option is
off by default and does not set an explicit output directory.
`system_greenhouse_bag.launch` is a playback/smoke entry, not a bag producer.

## Scenario: Enable Or Consume Runtime File Output

### 1. Scope / Trigger

- Enabling PCD saving, runtime logs, graph saving, screenshots/images, or bag
  recording.
- Promoting a generated file into a deployment or test asset.

### 2. Signatures

- FAST-LIO params: `pcd_save_enable`, `interval`, and
  `runtime_pos_log_enable`.
- Graph save command: `/save_file_dir`.
- Livox bag switch: `rosbag_enable`.

### 3. Contracts

- Runtime output must have a known owner, destination, overwrite policy, and
  retention policy.
- Do not write a new operational output beside source merely because an
  upstream package already does so.
- A generated map used for navigation must be promoted deliberately: record
  when/how it was produced, then validate the launch path that consumes it.
- A save path supplied over a ROS topic is still an external file-system
  boundary; validate it before widening its use.

### 4. Validation & Error Matrix

| Condition | Expected handling |
|-----------|-------------------|
| Output directory is absent or not writable | Surface an error; do not report success |
| Target file already exists | Confirm or document overwrite behavior |
| PCD save is enabled in a real-time run | Measure or explicitly accept the performance impact |
| Generated map is ignored by Git | Verify its presence separately on every deployment |
| Save command targets a versioned planner asset | Stop and protect the baseline before running |
| Bag recording has no output directory | Record the effective working directory and filename |

### 5. Good / Base / Bad Cases

- Good: write to an explicit runtime/output directory, check success, and
  promote only reviewed artifacts.
- Base: greenhouse navigation disables FAST-LIO PCD saving.
- Bad: assume `pcl::io::savePCDFileBinary` or `ofstream.open()` succeeded
  without checking the resulting file.

### 6. Tests Required

- Exercise the exact save path with a temporary destination.
- Check file creation, nonzero size, parseability, and overwrite behavior.
- For maps, launch the consuming publisher/localizer against the promoted
  artifact.
- For bag recording, confirm the actual output filename and directory.

### 7. Wrong vs Correct

#### Wrong

- Add `*.txt`, `*.csv`, or `*.vgh` globally to `.gitignore`.
- Treat `src/FAST_LIO/PCD/scans.pcd` as guaranteed because the path appears in
  launch XML.

#### Correct

- Ignore only proven output paths or naming patterns, preserving versioned
  planner inputs.
- Check deployment presence because `*.pcd` is ignored and the default map is
  not tracked.

## Current Repository State

- `.gitignore` ignores `*.pcd`, `*.bag`, and `*.log`.
- It does not ignore all TXT, CSV, TIFF, or VGH files.
- `src/FAST_LIO/PCD/scans.pcd` exists locally at roughly 219 MB but is not
  tracked.
- `src/FAST_LIO_SUPER/PCD/scans.pcd` also exists locally and is not consumed
  by the current greenhouse mainline.
- `src/FAST_LIO_SUPER/PCD/scans.yaml` and `scans.pgm` are tracked, creating a
  risk that metadata/image and ignored PCD content drift independently.

## Primary Evidence

- `.gitignore`
- `src/FAST_LIO/src/laserMapping.cpp`
- `src/FAST_LIO/include/common_lib.h`
- `src/FAST_LIO/src/IMU_Processing.hpp`
- `src/FAST_LIO_LOCALIZATION/src/laserMapping.cpp`
- `src/FAST_LIO_SUPER/src/laserMapping.cpp`
- `src/boundary_handler/src/graph_extractor.cpp`
- `src/graph_decoder/src/decoder_node.cpp`
- `src/livox_ros_driver2/launch_ROS1/`
