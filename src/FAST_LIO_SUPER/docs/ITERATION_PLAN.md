# FAST-LIO Super — Iteration Plan

迁移 FAST-LIO2 → Super-LIO 风格的分阶段路线。每个阶段的完成标志都是可验证的
构建/运行时产物，不依赖主观判断。

**当前状态（Phase A 全部完成）**：
- 包骨架就绪（`fast_lio_super`）
- `slam_core` 接口就绪：`IMapManager`（含 `insert / insertNoDownsample /
  removeBoxes / drainRemoved / flattenInto / empty / size / validSize / cropAround /
  snapshot / configure / stats`）+ `INeighborSearch`
- `IkdTreeMap` 是 `IMapManager` 的 ikd-tree 后端，自持 `KD_TREE<PointT>`
- `laserMapping.cpp` **全部 KD_TREE 操作已走 slamMap() 接口**，全局
  `KD_TREE<PointType> ikdtree` 已删除
- `config/mid360.yaml` 有 `slam_core:` 段；`configure()` 在 `main()` 里生效

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

### B-1 OctVox 数据结构
- [ ] `include/slam/map/oct_vox_map.hpp` + `src/slam/map/oct_vox_map.cpp`
  - Robin Hood hash（`absl/container/flat_hash_map` 或自实现）
  - 每个 voxel 挂 8 个子体素代表点；增量均值更新
  - LRU 链表，`capacity` 触发淘汰
- [ ] `config` 追加 `map_backend: "octvox"` 的默认参数文件。

### B-2 HKNN 搜索
- [ ] `include/slam/nn/hknn_search.hpp` + impl
  - 子体素遍历顺序 LUT
  - 早停（当前第 k 候选 dist2 > 下一个 cell 下界）
- [ ] 单元测试：与 brute-force KNN 在随机点云上对比，topK 一致率 > 99%。

### B-3 MapIO
- [ ] `include/slam/io/map_io.hpp`，二进制头 + voxel dump
- [ ] CLI：`rosrun fast_lio_super map_dump /path/to/map.bin`
      （建图结束时触发，或独立节点订阅）

### B-4 Relocalizer
- [ ] `include/slam/reloc/relocalizer.hpp` + impl（先 scan-to-map ICP+先验）
- [ ] ROS 服务：`~/relocalize` 接收 `geometry_msgs/PoseWithCovarianceStamped`
- [ ] 与 `FAST_LIO_LOCALIZATION` 共存时，`greenhouse_nav` 侧择一启用。

### B-5 A/B 实验
- [ ] 跑 4 组配置：`fl2_baseline / fl2_star / fl2_octvox_sidecar / superlio_like`
- [ ] 指标：p99 latency、mem peak、ATE/RMSE、每公里地图体量、重定位成功率
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
