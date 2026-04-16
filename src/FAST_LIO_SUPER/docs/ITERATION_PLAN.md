# FAST-LIO Super — Iteration Plan

迁移 FAST-LIO2 → Super-LIO 风格的分阶段路线。每个阶段的完成标志都是可验证的
构建/运行时产物，不依赖主观判断。

**当前状态（Phase B-1 完成；Super-LIO 源码 vendored）**：
- 包骨架就绪（`fast_lio_super` + 新 pkg `slam_octvox`）
- `slam_core` 接口就绪：`IMapManager` + `INeighborSearch`
- 两个后端并存，运行时按 yaml `slam_core/map_backend` 切换：
  - `ikdtree` → `makeIkdTreeMap()`（Phase A）
  - `octvox`  → `makeOctVoxMap()`（Phase B-1，封装 Super-LIO vendored GPL 代码）
- 新包 `slam_octvox`：GPLv3 header-only，vendored `OctVoxMap.hpp` +
  `HKNN_list60_gem.h` + `tsl::robin_map`；C++17
- `fast_lio_super` 整体升 C++17，license 改 GPLv3
- 源码阅读记录见 `docs/SUPERLIO_REFERENCE_NOTES.md`

---

## Phase A：接口抽象 + ikd-tree 包装（不改语义）

**目标**：让 FAST-LIO2 的 IESKF 经由 `slam_core` 间接访问 ikd-tree，行为与原版
字节级一致。

### A-1 接口层落地（DONE）
- [x] 新建 `fast_lio_super` 包，迁移原 FAST_LIO 内容
- [x] 写 `include/slam/{common_types,nn/neighbor_search,map/map_manager}.hpp`
- [x] 写 `include/slam/map/ikd_tree_map.hpp` + `src/slam/map/ikd_tree_map.cpp`
- [x] 新增 `fast_lio_super_slam` 静态库 target（与 exec 并列构建）

### A-2 IESKF 切到接口（DONE）
- [x] 残差 knn `Nearest_Search(...)` → `slamMap().search().knn(...)`
- [x] 平面拟合 `esti_plane(...)` 保留原实现（按计划一期只换 knn）

### A-2.b 地图操作全量迁移（DONE）
- [x] `IMapManager` 扩展：`insert / insertNoDownsample / removeBoxes / drainRemoved
      / flattenInto / empty / size / validSize`
- [x] `IkdTreeMap` 实现以上全部方法（封装 `Add_Points / Delete_Point_Boxes /
      acquire_removed_points / flatten / Root_Node / validnum / size`）
- [x] `laserMapping.cpp`：全局 `ikdtree` 已删除；`slamMap()` 返回 `IkdTreeMap&`
      - `acquire_removed_points` → `drainRemoved`
      - `Delete_Point_Boxes` → `removeBoxes`（BoxPointType → BoundingBox 转换）
      - 双 `Add_Points` → `insert` + `insertNoDownsample`
      - `Root_Node == nullptr` → `empty()`
      - `Build` → `insert`（首次插入触发内部 Build）
      - `validnum / size` → `validSize() / size()`
      - `flatten + PCL_Storage` → `flattenInto`
- [x] `fastlio_super_mapping` link `fast_lio_super_slam` 静态库

### A-3 配置外化（DONE）
- [x] `config/mid360.yaml` 增 `slam_core:` 段
- [x] `laserMapping.cpp` 启动时读取并调用 `slamMap().configure(mc)`
- [ ] 其它 yaml（`avia / horizon / velodyne / ouster64 / marsim`）按需复制同段

### A 残项（可选优化）
- [ ] `IkdTreeMap::cropAround` 目前 no-op；南方丘陵长时运行时可按需实现为
      "keep box 反向删除"
- [ ] `IkdTreeMap::raw()` 保留以备调试，Phase B 前删除
- [ ] 数值对比回归：同一 bag 下 `fast_lio` vs `fast_lio_super` 的 `/Odometry`
      位置差 < 1e-4 m（无 bag 环境下仅做编译验证）

**退出条件**：A-2.b 完成（全部 raw 路径收敛到 IMapManager）+ 数值对比通过 + 所有 yaml 配置生效。

---

## Phase B：OctVox + HKNN + Relocalizer

**目标**：用 Super-LIO 风格的地图/搜索/重定位替换 ikd-tree，可按 yaml 开关回退。

### B-1 OctVox 后端（DONE，via vendored Super-LIO）
- [x] 新建 header-only catkin 包 `slam_octvox`（GPLv3）
- [x] Vendored：`OctVoxMap.hpp` + `HKNN_list60_gem.h` + `tsl/robin_*.h`
- [x] `include/slam/map/oct_vox_map.hpp` + `src/slam/map/oct_vox_map.cpp`
      适配器实现 `IMapManager` + `INeighborSearch`
- [x] `laserMapping.cpp` 启动时按 `slam_core/map_backend` 选择后端
- [x] `config/mid360.yaml` 已支持 `map_backend: "ikdtree" | "octvox"`
- [x] C++17 全包升级；license 改 GPLv3
- [x] Docker 编译验证通过（`fastlivo-dev:noetic-ros-base`）

### B-1.b 语义遗留（仍待）
- [ ] OctVox 后端**不支持** box-delete / drainRemoved / cropAround（no-op）；
      A/B 时 FAST-LIO2 的 `cub_needrm` 计算时间会被吞掉，但不影响正确性
- [ ] `insertNoDownsample` 当前降级为 `insert`；若下游需要真正无损通道，
      需要扩展 Super-LIO backend
- [ ] 平面拟合走接口 `fitPlane`，但 `laserMapping.cpp` 目前仍调用原
      `esti_plane`（Phase A 决策）；若要用 OctVox 的 `calc_plane_coeff`，
      需要单独迁移这一步

### B-1.c 并发（DONE）
- [x] `IkdTreeMap::knn` / `OctVoxMapAdapter::knn` 移除 `std::mutex stats_mu_`，
      计时改用 `std::atomic<uint64_t>` + `std::memory_order_relaxed`
- [x] `OctVoxMapAdapter::size/validSize/empty` 改用 vendored 新增的 cheap
      `OctVoxMap::voxel_count()`（O(1) 读 `data_.size()`），不再每次调
      `getMap()` 做 O(N) 全表拷贝
- [x] `h_share_model` 的 OMP 并发度从编译期固定的 `MP_PROC_NUM=3` 改为
      yaml `slam_core/omp_threads`（0=默认 / <0=max / >0=显式）
- [x] 原生 `#pragma omp parallel for` 已在 FAST-LIO2 第 673 行，本次改动
      让它在 OctVox 后端下不再被 stats_mu 串行化
- [ ] **待用户验证**：在多核机器上 `omp_threads=8` 对比 `0` 的 p99 latency
      降幅（无 bag 环境不做）

### B-2 HKNN 调优（DONE 来自 vendored，后续可调）
- [x] Group 分层 + 早停 + mirror + XOR 优化（`HKNN_list60_gem.h` 直接用）
- [ ] 单元测试：与 `getTopK_VN`（朴素 19-邻居）对比 topK 一致率 > 99%
- [ ] 不同 `resolution` 下 `orders_min_dis2` 的重算（默认只对 0.5m 调过）

### B-3 MapIO
- [ ] `OctVoxMap::getMap(vector<float>)` 已 vendored，可直接 `pcl::io::savePCD`
- [ ] `OctVoxMap::resetMap(vector<float>)` 已 vendored（带 10 帧稳定期）
- [ ] `IMapIO` 接口暴露 + ROS 服务入口（B-3.b）

### B-4 Relocalizer
- [ ] 参考 Super-LIO `super_lio_reloc.cpp`（未 vendored）实现 `IRelocalizer`
- [ ] ROS 服务：`~/relocalize` 接收 `geometry_msgs/PoseWithCovarianceStamped`
- [ ] 与 `FAST_LIO_LOCALIZATION` 共存时，`greenhouse_nav` 侧择一启用

### B-5 A/B 实验
- [ ] 至少跑 3 组配置：`ikdtree / ikdtree+parallel / octvox`
- [ ] 指标：p99 latency、mem peak、ATE/RMSE、每公里地图体量、重定位成功率
- [ ] 参数对齐表：`voxel_size = 0.5` 时 ikdtree vs OctVox 的"有效代表点
      密度"不同，需在报告里单独说明（OctVox sub-voxel = 0.25m）
- [ ] 输出报告到 `docs/benchmarks/`。

**退出条件**：`superlio_like` 在 bag 回放与真实 mid360 上均不差于 baseline，
且 p99 latency 改进 ≥ 30%。

---

## Phase C：后端与多会话

**目标**：把"地图管理"升级到"带后端的 SLAM 平台"。

### C-1 Keyframe pose graph
- [ ] `include/slam/graph/keyframe_manager.hpp`
- [ ] 基于 GTSAM/Ceres 的 2-step：relative factor + loop closure
- [ ] 触发条件：位移 > 1m 或角度 > 10°
- [ ] 不阻塞 IESKF：后台线程异步优化，完成后刷 active map origin。

### C-2 Local BA
- [ ] 滑窗式平面/边特征 BA（VoxelMap 风格）
- [ ] 仅当 Phase B 全量通过后开启。

### C-3 多会话
- [ ] 建图 session 标识 + 地图合并（load + transform + insert）
- [ ] 与 `FAST_LIO_LOCALIZATION` 的 `/initialpose` 流程对齐。

**退出条件**：在温室长时间作业（> 30min）下 map size 稳定、闭环误差可观察收敛。

---

## 验证与回归策略

每个 PR 都应给出：
1. 改了哪些文件
2. `catkin_make` 是否通过（用 `fastlivo/ros:noetic-ros-base-focal`）
3. 对前一版本的行为影响（数值/性能/可观测指标）
4. 剩余风险

**Phase A 回归基线**：`src/FAST_LIO/` 原包保持不动，任何时候都能
`roslaunch fast_lio mapping_mid360.launch` 拿到基线对比。

---

## 南方丘陵专项待办（跨阶段）

- [ ] `MapConfig` 预留 `resolution_policy` 字段（固定/自适应），暂时只实现固定
- [ ] 接入方式：`MapResolutionPolicy::onScene(stats) -> new voxel_size`
- [ ] 近距树干/岩壁/建筑边缘保真，远距植被/叶片回波可强压缩（通过 curvature
      阈值在 `insert` 时分流，B-1 阶段附加）
