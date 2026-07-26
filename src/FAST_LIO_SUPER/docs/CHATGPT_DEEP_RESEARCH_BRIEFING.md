# FAST-LIO Super 后续演进：深度调研 Briefing

**目的**：本文件是交给 ChatGPT 做深度调研用的**完整上下文包**。读者不需要访问我们的代码仓库也能理解问题全貌。报告末尾列出 20+ 个具体调研问题，允许开放式答复，不要预设方案。

**写作日期**：2026-04-18
**项目**：`fast_lio_super`（FAST-LIO2 的演进分支，WSL2 + ROS1 Noetic + Ubuntu 20.04）
**当前状态**：Phase A/B-1/B-1.c 完成；Phase C（后端）尚未起步。

---

## 0. 快速背景

我们在把 **FAST-LIO2**（港大 MaRS 实验室的 IESKF-based LIO，只有前端）演进成一个完整的 SLAM 系统。同时吸收 **Super-LIO**（MaRS 最新的 LIO 变体，`arXiv:2509.05723`，2025）的地图管理（OctVoxMap + HKNN）。

**两者的关系：**
- FAST-LIO2 = IESKF 紧耦合前端 + ikd-tree 地图。BSD-3。
- Super-LIO = 同类型前端（IESKF），但把 ikd-tree 换成 OctVoxMap（8 sub-voxel/voxel + running-mean + LRU）+ HKNN（分层近邻搜索）。**没有后端**（我们实测确认源码里无 keyframe/pose_graph/loop_closure/gtsam 任何符号）。GPLv3。
- 两者都**不是 SLAM 系统**，只是"更强的 LIO 前端"。

**我们的长期目标场景**：
- 南方温室（结构化、小尺度，< 30m × 30m）
- 南方林地 / 丘陵（半结构化，稀疏点云，回环覆盖差，视角复访差）
- 传感器：**Livox Mid-360**（窄 FoV 40°、非重复扫描、稀疏、带 IMU）
- 平台：无人小车 / 农机底盘，不是车载高速
- 未来可能扩到：多机、多会话、长期地图（跨季节）、体素导航（voxel navigation / ESDF / traversability）

---

## 1. 当前工程状态

### 1.1 包结构

```
src/
  FAST_LIO/                ← 原版 FAST-LIO2，保留作为 baseline
  FAST_LIO_SUPER/          ← 我们的演进分支（= FAST-LIO2 复制 + 接口化）
  slam_octvox/             ← 新建 header-only 包，vendored Super-LIO 的 OctVoxMap + HKNN + tsl::robin_map
  livox_ros_driver2/       ← Mid-360 驱动
  ikd-Tree/                ← BSD，FAST-LIO2 的依赖
  其它 nav 相关包（far_planner、terrain_analysis、...）
```

### 1.2 Phase A（完成）— 地图层接口抽象

在 `FAST_LIO_SUPER/include/slam/` 下抽出了两个接口：

```cpp
// include/slam/nn/neighbor_search.hpp
struct Query { Eigen::Vector3f center; int k; float max_dist2; };
class INeighborSearch {
  virtual int knn(const Query& q, std::array<Neighbor, 8>& out) const = 0;
  virtual bool fitPlane(const Query& q, PlaneCoef& out) const = 0;
};

// include/slam/map/map_manager.hpp
class IMapManager {
  virtual void configure(const MapConfig& cfg) = 0;
  virtual void insert(const PointVec& world_pts) = 0;
  virtual void insertNoDownsample(const PointVec& world_pts) = 0;
  virtual int  removeBoxes(const std::vector<BoundingBox>& boxes) = 0;
  virtual void cropAround(const Eigen::Vector3d& c, float radius) = 0;
  virtual void drainRemoved(PointVec& out) = 0;
  virtual void flattenInto(PointVec& out) const = 0;
  virtual CloudPtr snapshot(float voxel) const = 0;
  virtual bool empty() const = 0;
  virtual std::size_t size() const = 0;
  virtual std::size_t validSize() const = 0;
  virtual MapStats stats() const = 0;
  virtual INeighborSearch& search() = 0;
  virtual const char* name() const = 0;
};

std::unique_ptr<IMapManager> makeIkdTreeMap();   // BSD-3 后端
std::unique_ptr<IMapManager> makeOctVoxMap();    // GPLv3 后端（Super-LIO vendored）
```

`laserMapping.cpp`（FAST-LIO2 主节点）所有 `ikdtree.*` 调用被替换成 `slamMap().*`，运行时按 `slam_core/map_backend` yaml 选择。行为与原版 FAST-LIO2 字节级一致（未在 bag 上回归，只做了编译验证）。

### 1.3 Phase B-1（完成）— Super-LIO vendored

- `slam_octvox` 包 header-only，GPLv3。
- 把 Super-LIO 源码的 `OctVoxMap.hpp`、`HKNN_list60_gem.h` 直接 vendored 进来（未改核心逻辑，只删掉 `<execution>` / `<filesystem>` / `saveMap()` 等不需要的部分，并加了 `voxel_count() / capacity() / resolution()` 三个 O(1) getter，避免 stats 路径做 O(N) 扫描）。
- `tsl::robin_map`（MIT）vendored 进 `slam_octvox/include/tsl/`。
- 整包 C++14 → C++17（Super-LIO 用了 `if constexpr`、structured bindings）。
- `fast_lio_super` 的 license 改为 GPLv3（由 OctVox 的 GPLv3 传染）。
- Docker 镜像 `fastlivo-dev:noetic-ros-base` 编译通过。

### 1.4 Phase B-1.c（完成）— 并发修正

- `OctVoxMapAdapter::knn` / `IkdTreeMap::knn` 热路径里的 `std::mutex stats_mu_` 改成 `std::atomic<uint64_t>` + `std::memory_order_relaxed`。原因：FAST-LIO2 的 `h_share_model` 用 `#pragma omp parallel for` 并发跑残差构建，之前的 mutex 把 OMP 串行化了。
- FAST-LIO2 编译期常量 `MP_PROC_NUM=3` 改成运行时 yaml `slam_core/omp_threads`（0=默认 3 / <0=max / >0=显式）。

### 1.5 未完成的遗留（B-1.b / B-1.d / B-2/3/4）

- OctVox 后端**不支持** `removeBoxes` / `drainRemoved` / `cropAround`（no-op）。Super-LIO 用 LRU 承担地图体积控制，语义不等价于 FAST-LIO2 的 FoV 立方体删除。
- `insertNoDownsample` 当前降级为 `insert`。Super-LIO 没有"无损通道"。
- `laserMapping.cpp` 的 `esti_plane` 没迁到 `fitPlane` 接口（Phase A 决策：只换 knn）。
- HKNN 的 `orders_min_dis2` LUT 默认只为 `resolution=0.5m` 调过，其它 voxel_size 需要重算。
- MapIO / Relocalizer 没开始。

### 1.6 代码量参考

| 模块 | 行数 |
|---|---:|
| `FAST_LIO_SUPER/src/laserMapping.cpp` | ~1500（基本同原 FAST-LIO2） |
| `FAST_LIO_SUPER/src/slam/**.cpp` | ~400 |
| `FAST_LIO_SUPER/include/slam/**.hpp` | ~250 |
| `slam_octvox`（vendored） | ~2000（含 HKNN LUT） |

---

## 2. 主线目标：我们想走向哪里

**一句话**：把 FAST-LIO2 从"前端"演进成一套面向温室/林地/丘陵的**长期可运维 SLAM 平台**，具备：

1. **后端**：keyframe + pose graph + loop closure + 异步优化
2. **长期地图**：跨 session 保存/载入、prior-map localization、多会话合图
3. **规划友好地图**：occupancy / ESDF / traversability（未来接 voxel navigation）
4. **Super-LIO 的地图管理**作为前端 active map 的选项（已做）
5. **多后端可切**：至少保留 ikd-tree baseline 可回退

**非目标**（至少第一期不做）：
- 视觉融合（FAST-LIVO 路线）
- GPU 加速
- 高速车载场景（KITTI、M2DGR 精度冲榜）
- 多 LiDAR

---

## 3. 已知的候选系统（仅列过已看过源码或论文/README 的）

### 3.1 FAST-LIO2 社区魔改生态（第一优先级候选族群）

| 仓库 | 特点 | License | 实测过？ |
|---|---|---|---|
| `gisbi-kim/FAST_LIO_SLAM` | FAST-LIO2 + SC-PGO（ScanContext + GTSAM）sidecar | Apache-2.0？未确认 | 未 clone |
| `engcang/FAST-LIO-SAM` | FAST-LIO2 + LIO-SAM 的 mapOptimization 模块化移植 | 未确认 | 未 clone |
| `engcang/FAST-LIO-SAM-QN` | 上者 + Quatro（粗配准）+ Nano-GICP（精配准） | 未确认 | 未 clone |
| `engcang/FAST-LIO-SAM-SC-QN` | 再加 ScanContext | **CC-BY-NC-SA 4.0**（非商业） | 未 clone |
| `yanliang-wang/FAST_LIO_LC` | Loop-closure 版本，闭环结果会**回写前端位姿** | 未确认 | 未 clone |
| `zhh2005757/FAST-LIO-Multi-Sensor-Fusion` / `better_fastlio2` | 民间整合版 | 未确认 | 未 clone |
| `liangheming/FASTLIO2_ROS2` | ROS2 版 + PGO + 重定位 + 一致性地图 | 未确认 | 未 clone |
| `HViktorTsoi/FAST_LIO_LOCALIZATION` | 用先验地图做 localization-only | 未确认 | 仓库里有 |
| `rpng/VoxelMap_LIO_SAM` | Voxel 局部图改造，对 Mid360 友好 | 未确认 | 未 clone |
| `HViktorTsoi/VoxelMapPlus_FASTLIO2` | VoxelMap++ + STD + LRU + FAST-LIO2 | 未确认 | 未 clone |

**我们自己的观察**：这一族群的思路高度一致：**保留 FAST-LIO2 IESKF + ikd-tree 前端不动，把 keyframe / pose graph / loop closure 作为 sidecar 异步线程挂上去**。和我们当前的 `slam_core` 接口分层天然兼容。

### 3.2 完整系统类（更适合借模块/借图纸，不适合整包移植）

| 系统 | 论文 | 特点 | 主要依赖 | License | 对我们的角色 |
|---|---|---|---|---|---|
| LIO-SAM | Shan 2020 | GTSAM + ISAM2 + ICP loop | GTSAM（需源码编译） | BSD-3 | 后端骨架参考 |
| Voxel-SLAM | hku-mars 2024 | 自适应 voxel + 分层 BA + STD 回环，五模块完整 | Ceres + PCL，自研 voxel | **GPLv2-only**（license 兼容性风险） | 研究方向参考，不建议并库 |
| GLIM | Koide 2024 | GPU scan matching + 全局 trajectory 优化 | CUDA + GTSAM | Apache-2.0？未确认 | 高精度离线参考 |
| MOLA / MOLA-LIO | Blanco 系列 | "simple-map"（view-based keyframe store）+ 可派生多种 metric map | ROS2 + MRPT | BSD？未确认 | **系统架构参考**（最打动我们的部分） |
| KISS-SLAM | Vizzo 2024 | KISS-ICP + MapClosures + g2o，轻量 | 无大依赖 | MIT | 轻量后端参考 |
| hdl_graph_slam | Koide 2019 | g2o + GICP loop | g2o (apt) + fast_gicp + ndt_omp | BSD-2 | 纯独立后端，但与 FAST-LIO2 生态脱节 |
| LTA-OM | MaRS 2024 | Long-term association，multi-session | GTSAM + Ceres + TBB + STD | **许可证信号冲突**（README vs LICENSE 不一致） | 多会话参考，不建议并库 |
| LT-mapper | Kim 2022 | 多会话、终身地图 | FAST_LIO_SLAM + SC-LIO-SAM saver | 未确认 | 长期地图参考 |
| SuperLoc | CMU 2024 | Prior-map localization + **失败风险预评估**（优化前就评估 alignment risk） | 未确认 | 未确认 | Relocalizer 参考 |

### 3.3 规划/导航侧车地图候选（与后端正交，但属于主线目标 4）

- **nvblox**（NVIDIA）：实时 TSDF/ESDF，但需要 GPU
- **Bonxai / Rolling-Bonxai**：稀疏体素数据结构，CPU
- **G-VOM**：off-road 路径规划用 3D voxel
- **RoadRunner M&M**：dense forest 场景的 traversability / 高程预测
- **VoxelMap++ / C3P-VoxelMap / R-VoxelMap**：plane-merging persistent map

---

## 4. 我们已经形成的几点认识（可能是错的，请 ChatGPT 判断）

### 认识 A：Super-LIO 没有后端，也不适合承担长期地图

- OctVox 的 sub-voxel running-mean 只保留几何代表，丢掉了 intensity/time/ring/normal 等原始观测语义。
- LRU 机制意味着旧地图点会被"遗忘"，无法跨 session 恢复。
- 因此它适合做"前端 active registration map"，不适合做"唯一真源地图"。

### 认识 B：应该"三图分离"

- **Active map**（前端 scan-to-map）：OctVox / ikd-tree 二选一
- **Keyframe store**（后端与长期地图的真源）：view-based，保留原始 scan 或特征
- **Planning map**（下游导航消费）：occupancy / ESDF / traversability，异步从 keyframe store 派生

不应该把这三件事塞进同一个数据结构。

### 认识 C：南方林地的回环，ScanContext 可能不够

- 2024 年的一篇 dense forest LiDAR place recognition 研究（Logg3dNet 论文 or 类似）表明：在密林场景下 ScanContext 判别性显著下降，Logg3dNet（learned descriptor）显著胜出。
- 因此 `ILoopCandidateProvider` 必须插件化。至少预留 ScanContext（baseline）+ Logg3dNet/STD/BEVPlace++ 接口。

### 认识 D：Mid360 的窄 FoV 会放大回环问题

- Mid360 FoV 40° × 360°（水平环视但垂直窄）。
- 在非重复扫描下单帧点分布依时间积累，稀疏且不均。
- 传统基于 GICP scan-to-scan 的回环（hdl_graph_slam 路线）在 Mid360 上有效性存疑。
- 基于 submap-to-submap 或 global descriptor 的回环可能更稳。

### 认识 E：先 keyframe + PG + loop，再考虑 BA

- HBA / BALM / local BA 是"一致性地图"的后处理层，不是第一版后端该做的。
- 第一版后端只要解决"长时漂移 + 单次回环校正"就足够工程化落地。

### 认识 F：hdl_graph_slam 作主线不合适

- 它是"LOAM 时代"的独立后端，设计上与 LIO 生态脱节。
- apt 依赖齐全（g2o / fast_gicp / ndt_omp 都有 noetic 包）—— 这是它**唯一**的工程优势。
- 但它的 scan_matching_odometry 是自带的，和 FAST-LIO2 并行运行会冗余；只用它的 graph 部分又要改很多代码。
- 做 fallback 可以，做主线方向偏离生态。

### 认识 G：GTSAM 源码编译是一次性成本，值得付出

- Ubuntu 20.04 noetic 仓库没有 GTSAM。
- 但 GTSAM 4.0.3 源码编译 30min，且 LIO-SAM / FAST_LIO_SLAM / FAST-LIO-SAM / SC-QN / LTA-OM / GLIM 全部依赖它。
- 如果走 FAST-LIO2 生态后端，GTSAM 几乎无法绕开。
- docker 加一层是合理代价。

---

## 5. 仍然无法判断 / 存在分歧的问题

### Q1. 后端究竟该"抄哪一家"？

我上一轮报告推荐 hdl_graph_slam 做 MVP → LIO-SAM 做 v2。外部深度研究报告推荐 FAST-LIO-SAM-SC-QN 做图纸 + 自写。两者都有道理：
- **hdl_graph_slam 路线**：依赖零成本、独立 nodelet、~200 行 bridge 就能跑。但回环质量差，生态脱节。
- **FAST-LIO-SAM-SC-QN 图纸 + 自写**：贴近生态，模块化干净。但 SC-QN 本身是非商业许可证（CC-BY-NC-SA），不能并库，要重写；工程量 2000-3000 行。
- **FAST_LIO_SLAM 整包挂载**：最省工程量，但代码质量/维护活跃度未知。
- **其它**：better_fastlio2、FASTLIO2_ROS2 是否足够稳定？

我们没有足够信息在这四条路线间做出有依据的选择。

### Q2. 许可证到底有多大约束？

项目未来可能：
- 商业化（南方农机/温室产品）
- 开源（但不确定是否纯 GPL）

目前 fast_lio_super 已被 OctVox 的 GPLv3 传染。现在的问题：
- GPLv3 传染会不会让我们将来无法整合某些 BSD/Apache 的最新改动？
- CC-BY-NC-SA 4.0（SC-QN）的"非商业"条款在**产品未商业化前**是否允许集成？商业化时再重写？
- LTA-OM 的 README 写 MIT 但 LICENSE 文件是 GPL —— 这类信号冲突的仓库能用吗？
- MOLA 的商业化边界到底在哪？

我们没有法务，只能靠公开资料判断。

### Q3. 第一版后端的"最小可行切面"到底是什么

可选的最小集：
- **方案甲**：只做 keyframe + 相邻 BetweenFactor + iSAM2 增量优化（**不做回环**）。只解决"接口通路"问题。
- **方案乙**：甲 + ScanContext 回环 + ICP 验证。第一版系统化 SLAM 就位。
- **方案丙**：乙 + Nano-GICP 替换 ICP + MapCorrectionBridge（回环后外层 TF 补偿）。
- **方案丁**：丙 + MapIO（save/load）+ Relocalizer。

每升一级都是几百到上千行代码。第一版选哪个档位最能"证明系统通路且不提前烧工程预算"？

### Q4. 与现有 `greenhouse_nav` 主线的集成方式

- greenhouse_nav（远地导航栈）消费 `/Odometry` 和点云。
- 加入后端后，回环会导致"map jump"（历史位姿跳变）。
- 解决方案有两类：
  - 在 bridge 层加 `T_map_odom` 补偿 TF（前端 odom 不跳）
  - 前端直接接受 PG 优化后的位姿（需要修改 FAST-LIO2 主线状态更新逻辑）
- FAST-LIO-SAM-SC-QN 这类仓库是怎么处理的？是否值得抄？

### Q5. 温室 vs 林地两种场景是否需要不同参数档

- 温室（30m × 30m）：高回环频率、结构化、小关键帧阈值（0.3m）
- 林地（> 100m × 100m）：低回环频率、半结构化、大关键帧阈值（1-2m）

是做成一个 yaml 两套参数，还是抽象成 `scene_profile` 层？Super-LIO 的 `resolution_policy` 设计已经有类似预留，但没实现。

### Q6. Mid360 专项

- 如何在稀疏点云下做回环验证？ICP/GICP 的 inlier 阈值要调到多少？
- 是否应该用 **accumulated submap**（N 帧积累成子图）做回环，而不是单帧 scan？
- Livox 的非重复扫描特性在 PGO 节点应如何建模？

### Q7. "三图分离"在工程上怎么落地

- Active map 与 Keyframe store 之间是什么关系？Keyframe 快照时是从 active map 截取还是单独存原始 scan？
- Keyframe store 要不要自带 ID / 时间戳 / 位姿 / descriptor 索引？
- Planning map sidecar 是从 keyframe store 异步派生还是从前端点云旁路抽取？

### Q8. 我们是否应该现在就向 ROS2 迁移？

- ROS1 Noetic EOL 2025-05，已经过期。
- 但 FAST-LIO2 生态的大部分仓库仍在 ROS1。
- FASTLIO2_ROS2、GLIM、MOLA、KISS-SLAM 都已经在 ROS2。
- 现在加后端时候同步做 ROS2 迁移 vs 先做 ROS1 跑通再迁移，哪个总成本低？

---

## 6. 需要 ChatGPT 深度调研的具体问题

请 ChatGPT 针对以下每个问题，**基于 2023–2026 最新公开资料（论文、官方仓库、issue tracker、社区实测）**给出回答。允许分歧、允许未知，不要强行合成"最优方案"。

### A 组：候选系统深度对比

1. **FAST_LIO_SLAM / FAST-LIO-SAM / FAST-LIO-SAM-QN / FAST-LIO-SAM-SC-QN / FAST_LIO_LC / better_fastlio2 / FASTLIO2_ROS2** 这些民间整合版，它们之间的差异是什么？分别的代码质量、维护活跃度、许可证、已知 issue、Mid360 支持情况？
2. 这一族群里，哪几个是最适合我们这样"已经有 IESKF 前端 + 地图接口，只想接后端"的项目借用的？是整包挂载还是借图纸？
3. LIO-SAM 的 `mapOptimization.cpp` 在 2024-2026 有没有被某个社区分支做成"独立后端组件"（剥离前端、剥离 LOAM 特征提取）？如果有，哪个分支做得最干净？
4. **GLIM** 在 CPU-only 模式下精度如何？能否替代 LIO-SAM 的 backend 作为我们的后端主线？
5. **MOLA / MOLA-LIO** 的 "simple-map" 概念在实际工程里的优势有没有被验证？有没有社区项目实际基于它做出商用产品？
6. **KISS-SLAM / MapClosures** 组合的回环质量，在稀疏 LiDAR（Mid-360 级别）上有实测案例吗？

### B 组：Mid360 专项

7. 2024-2026 有没有**专门针对 Mid360 / 非重复扫描 LiDAR**的回环检测方法？
8. Mid360 窄 FoV 下，scan-to-scan GICP 回环的实际成功率是多少？accumulated-submap 回环是否更稳？
9. Livox `fast_lio` 社区对于 Mid360 的调参经验（keyframe 阈值、voxel 大小、回环半径）有没有公开的 reference config？

### C 组：回环检测（Place Recognition）

10. 2024-2026 在稀疏 LiDAR / 密林场景下，**ScanContext / ISC / M2DP / STD / BTC / Logg3dNet / BEVPlace++** 的横评结果如何？
11. **Logg3dNet** 和 **BEVPlace++** 的部署成本（模型大小、推理时延、是否需 GPU）？CPU-only 能接受吗？
12. **STD (Stable Triangle Descriptor)** 作为 CPU-only 的 handcrafted descriptor，在温室/林地场景的表现？

### D 组：长期地图 / Localization-only

13. **LT-mapper** / **LTA-OM** / **MOLA 多会话** / **SuperLoc** 这几条长期地图路线，哪个工程落地成熟度最高？
14. "先验地图 + 实时 localization"模式在 FAST-LIO2 生态有没有标准实现？FAST_LIO_LOCALIZATION 和 FAST-LIO-Localization-SC-QN 的区别？
15. 跨 session 地图合并（同一地点多次建图合并成一张图）目前有没有成熟开源工具？

### E 组：规划地图 sidecar

16. 从 SLAM keyframe store 派生 **occupancy / ESDF / traversability** 的 CPU-only 方案（不用 nvblox）有哪些？**Bonxai / Rolling-Bonxai / G-VOM** 的实际部署经验？
17. 温室场景（低矮、小尺度、结构化）vs 林地场景（半结构化、高遮挡），planning map 的设计是否要分别处理？

### F 组：许可证与商业化

18. 下列仓库的**实际 license**（以 repo 根目录 LICENSE 文件为准）和商业化约束：
    - FAST_LIO_SLAM、FAST-LIO-SAM、FAST-LIO-SAM-QN、FAST-LIO-SAM-SC-QN、FAST_LIO_LC
    - better_fastlio2、FASTLIO2_ROS2
    - Voxel-SLAM、LTA-OM、GLIM、MOLA
    - VoxelMap++、C3P-VoxelMap、R-VoxelMap
19. 我们当前 fast_lio_super 是 GPLv3（来自 OctVox 传染）。这是否会阻止我们集成上述 BSD/MIT/Apache 的最新改动？未来若要商业化，GPLv3 的"copyleft 传染"具体约束是什么？
20. 哪些候选的许可证对"商业闭源产品"是**完全安全**的（BSD/MIT/Apache-2.0），哪些是**需要合规成本**的（LGPL/MPL），哪些是**基本无法用**的（GPL/AGPL/CC-NC）？

### G 组：系统架构

21. 在"FAST-LIO2 + 外挂后端"拓扑里，**回环后的 map correction**（"map jump"）工程上有几种处理方式？社区主流做法是什么？
22. Keyframe 触发条件在**温室 (< 30m)** 和**林地 (> 100m)** 这两种尺度下差距很大，有没有成熟的自适应方案？
23. 后端异步线程与前端 IESKF 线程之间的数据流范式（SPSC queue / shared pointer / double buffer），哪种在生产代码里更稳？
24. 多后端 / 多地图 backend 的抽象层，除了我们当前的 `IMapManager` 接口，**MOLA / KISS-SLAM / GLIM** 还给出过什么样的抽象？

### H 组：ROS1 vs ROS2

25. FAST-LIO2 的 ROS2 原生版本 (`FASTLIO2_ROS2` 或其它) 目前的成熟度？是否可以直接替代 ROS1 版？
26. Mid360 的 livox_ros_driver2 在 ROS2 下的成熟度？有没有已知 bug？
27. 如果现在就做 ROS2 迁移，整个主线（前端 + 后端 + 导航 + 重定位）的工程量比"先跑通 ROS1 再迁"多出多少？

### I 组：我们没想到的

28. **有没有我们完全忽略、但 2024-2026 出现的更好的 LIO + backend 路线？**（例：Point-LIO、Faster-LIO、COIN-LIO、PV-LIO、iG-LIO、Adaptive-LIO、DV-LIO、LOG-LIO2、VOX-LIO 等是否值得看？）
29. **有没有哪条路线能让我们直接跳过"keyframe PGO"经典设计，上更现代的方案**（例如 implicit map / PIN-SLAM / SHINE-Mapping）？风险与回报？
30. **如果只允许我们选一条路线在 3 个月内让 fast_lio_super 具备"可运维的 SLAM 系统"的全部能力，你会选哪条？为什么？**

---

## 7. 附：我们已读过的内部文档

- `docs/ARCHITECTURE.md` — 模块分层
- `docs/ITERATION_PLAN.md` — Phase A/B/C 路线
- `docs/SLAM_CORE_API.md` — 接口契约
- `docs/SUPERLIO_REFERENCE_NOTES.md` — 实际从 Super-LIO 源码读到的内容
- `docs/BACKEND_SELECTION_REPORT.md` — 上一轮狭窄选型报告（LIO-SAM / Voxel-SLAM / hdl_graph_slam 三选一）

## 8. 附：我们的约束清单

- **平台**：WSL2 + Ubuntu 20.04 + ROS1 Noetic（docker `fastlivo-dev:noetic-ros-base`）
- **编译器**：gcc-9 / C++17
- **CPU**：未指定（假设 8 核级别）
- **GPU**：无（不接受 CUDA 依赖作为主线；可作为离线后处理的可选项）
- **传感器**：Livox Mid-360（主）、备选 Velodyne/Ouster（低优先级）
- **许可证**：目前 GPLv3。可以接受 GPL，但要明确**哪些是传染源 / 哪些是商业化障碍**。
- **人力**：1-2 人，3 个月时间窗
- **数据**：**目前没有 bag**。所有改动只做编译验证，数值回归依赖用户在真机上跑。

---

## 9. 产出期望

请 ChatGPT 对上述 30 个问题给出**尽可能完整**的答复。结构建议：

- 每个问题单独一节
- 答复里明确区分「事实」「推断」「未知」
- 引用具体论文 arXiv 号、github 仓库名、issue 编号、官方文档 URL
- 不要合成「最优方案」—— 我们想看的是**选择面**与**各路线的真实风险**，而不是你替我们做决定
- 最后一节允许给出 ChatGPT 自己的综合判断，但必须明确标注「这是我的个人综合判断」

**禁止**：
- 编造仓库名、论文名、arXiv 号
- 把"推断"写成"事实"
- 用"据称""可能""也许"掩盖未知
- 忽视 2024-2026 的最新进展，只引用 2020-2022 的经典方案

如有必要，可以指出「这个问题需要在我们的真实 bag 上实测才能回答」——我们能接受"这部分不能只靠文献判断"的回复。

---

**end of briefing**
