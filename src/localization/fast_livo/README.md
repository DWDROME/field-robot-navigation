# FAST-LIVO2

本包维护 FAST-LIVO2 完整源码，提供地图预算、图像冷热存储、输入队列和退出生命周期管理。ROS 1 包名 `fast_livo`，程序 `fastlivo_mapping`，节点 `laserMapping`；topic、frame 和 Livox 消息身份保留。

## 来源与许可

基于 [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) 的提交 `0d2c0346107b75b59934975adec9a6eeeb913c64`。本库修改覆盖地图容量、图像存储、队列和资源释放。Sophus、vikit 的固定版本见 [ros1.repos](../../../third_party/ros1.repos)。

`provenance/` 保存原始声明：FAST-LIVO2 LICENSE 与 README 声明 GPLv2，package.xml 声明 BSD；vikit manifest 声明 GPLv3，Sophus 文件头采用 MIT 风格授权。使用和分发须按实际涉及文件及依赖的许可处理。

## 构建与运行

需要 Git、PyYAML、ROS Noetic、CMake、GCC 9、Eigen、OpenCV、PCL 及 package.xml 所列依赖。

```bash
bash tools/build-fastlivo2.sh .tmp/fastlivo-build
source .tmp/fastlivo-build/devel/setup.bash
roslaunch fast_livo mapping_avia.launch
```

在仓库根目录执行；首次准备固定 Sophus/vikit，后续增量编译。按传感器与相机选择、修改包内 launch/config。内存参数可通过 `rosparam load src/localization/fast_livo/config/memory.yaml` 加载到根命名空间，具体语义见下文。

附加 `check` 执行确定性生命周期测试，`FASTLIVO_MEMORY_SANITIZE=ON` 启用 ASan/LSan。`test/lsan-noetic.supp` 记录依赖初始化的排除项。上游的可选 PCD、图像和 COLMAP 输出写入包内被忽略的 `Log/`；COLMAP 初始化会清空该次导出的 images 与 sparse/0 目录。

## Runtime configuration

Use the upstream launch and calibrated sensor/camera configuration. Load `config/memory.yaml` into the same root parameter namespace as the upstream `common`, `vio` and `lio` settings, after their defaults. Camera intrinsics retain their upstream `laserMapping/...` names. LIVO requires `imu/imu_en: true`. Set `memory/cold_parent` to an existing writable directory outside the generated `fast_livo` source directory. The default is `/tmp`; choose a disk-backed parent when cold data must not consume tmpfs memory.

All memory parameters are read once at startup; online lowering is unsupported. Invalid finite-positive limits, malformed extrinsic arrays or invalid patch/layer ranges fail startup. Defaults are engineering budgets, not demonstrated real-time or RSS targets.

| Parameters | Unit and effect |
| --- | --- |
| `lio_voxels`, `vio_voxels` | Root map entries; serial exact LRU victim selection, O(number of roots) per eviction |
| `visual_points`, `points_per_voxel`, `observations_per_point` | Total visual points, points per root, observations per point |
| `visual_radius` | Axis-aligned half-width in metres, independent visual spatial trimming; LIO keeps upstream `local_map` sliding settings |
| `hot_bytes`, `image_bytes` | Managed grayscale pixels and maximum single managed image |
| `cold_bytes`, `cold_files`, `image_records` | Reserved cold payload including headers, file count, metadata count |
| `write_jobs`, `write_bytes` | Queued plus actively writing tasks/bytes, one writer thread |
| `input_messages`, `imu_messages` | ROS callback queues and internal camera/LiDAR or IMU queue lengths |
| `input_points`, `input_bytes` | Maximum single raw cloud points/message bytes; converted BGR image also bounded before conversion |
| `accumulation_points`, `path_poses` | Synchronized cloud and optional output accumulation capacity; trajectory tail size |
| `max_imu_gap` | Maximum accepted interval in seconds; clock reversal, gap and overflow invalidate the synchronization epoch |

Input queue memory is bounded by message count times maximum message size, not `hot_bytes`. Camera RGB, debug images, PCL working buffers, ROS transport, fixed-size state matrices and allocator overhead are separate finite resources. Map tree bounds also depend on validated upstream `lio/max_layer` and `lio/max_points_num`. These counters are not total process RSS.

## Ownership, failure and shutdown

LIO roots own trees and planes. VIO roots own visual points, points own Features, and Features own patches plus shared stable image IDs. `SubSparseMap` and retrieval arrays borrow points only within a frame. Actual root reads/updates atomically mark access and pin them. OMP residual construction never modifies map structure. Serial insertion evicts the oldest unpinned root; if none exists, the new resource is rejected and counted. Capacity trimming and spatial trimming are separate operations. Current-frame output images remain alive for mapper publication after VIO returns.

An image ID has one managed immutable grayscale allocation. Cold writes reserve file/task/byte capacity before scheduling. Successful header/pixel write, fsync and close establish a cold copy; pixels are dropped only without active readers. Readers hold explicit leases and concurrent loads of the same ID share one allocation. A length/header/checksum failure marks the record unavailable; warp retrieval skips that observation before assembling aligned residual arrays. Inverse composition reuses those accepted leases for all pyramid levels; optional debug reads also acquire leases. Failed writes preserve hot pixels. No zero replacement image is produced.

Each run gets `fastlivo2-XXXXXX` under the configured parent. Raw `.livo` files are lossless, same-run/native-endian records, not a portable dataset format. `owned.txt` is a bounded, atomically replaced inventory of created file names and inode identities; `owned.next` is the identifiable temporary manifest. `cold_bytes` counts image headers/payload and outstanding reservations; the two bounded manifests and filesystem block overhead are additional. Cleanup uses a directory descriptor, no-follow operations and identity checks. Foreign entries and replaced paths are preserved. Last identity/lease/task release deletes only registered ordinary image files. Failed deletions retain accounting and diagnostics. The small final manifest and run directory remain for inspection; there is no recursive directory deletion or startup sweep of previous runs.

Input overflow or IMU discontinuity clears paired queues and pending synchronization, suspends propagation until the next EKF state and discards the first incomplete integration interval while preserving the estimated state. Trajectory publishing retains a finite tail. Optional cloud accumulation drops an older pending batch before appending within capacity; a publish attempt with no successfully coloured point still releases that pending batch. Thus bounded mode cannot provide a complete historical trajectory/map export.

Shutdown stops subscriptions/timer first, releases frame borrows and observations, drains bounded writes and joins the writer, then releases remaining maps and queues. IDs/leases that outlive the store facade retain their valid storage until released. Startup, saturation, write/read/delete failures and periodic resource counters are visible in ROS/stderr logs; a cleanup failure is not reported as successful reclamation.

