# C-1 后端骨架图纸

**主骨架来源**：`~/refs/fast_lio_super/FAST-LIO-SAM/`（engcang，CC-BY-NC-SA 4.0，只作图纸不并库）
**融合来源**：`~/refs/fast_lio_super/FAST_LIO_SLAM/SC-PGO/`（gisbi-kim，license 模糊 → 只作图纸）
**日期**：2026-04-18（合并 SC-PGO 升级 v1.1）
**用途**：Phase C-1 后端最小可行骨架设计参考

**更新说明（v1.1，合并自 SC_PGO_COMPARISON.md）**：
1. Odom factor noise：区分 rot/trans（`1e-6 rot, 1e-4 trans`）
2. Loop factor noise：Cauchy robust kernel + 固定 variance，替代 fitness 各向同性
3. Keyframe 触发：累计双阈值（trans + rot），替代单次距离
4. Backend 订阅 topic：yaml 外化，通用前端
5. Loop candidate queue：backpressure 检查

---

## 0. 核心结论

FAST-LIO-SAM 是一个**独立 ROS sidecar 节点**，订阅 FAST-LIO2 的 `/Odometry` + `/cloud_registered`，自己维护 keyframe store + GTSAM pose graph，**完全不回写前端状态**。用 TF + `last_corrected_pose × odom_delta` 公式做实时 pose 补偿。总代码 1056 行（含可视化和 save_map）。

这正是 v2 选型报告推荐的路线。我们 C-1 可以**照这个拓扑做**，只是把 pcl::ICP 换成 small_gicp/nano_gicp。

---

## 1. 拓扑（ASCII 图）

```
FAST-LIO2 节点（我们已有的 fast_lio_super）
  ├─ 发布 /Odometry                    ───┐
  └─ 发布 /cloud_registered            ───┤   （topic 名字 yaml 外化，SC-PGO 升级）
                                          ▼
                              [fast_lio_sam_backend 新节点]
                                          │
        ┌─────────────────────────────────┼──────────────────────────┬────────────────┐
        │                                 │                          │                │
  odomPcdCallback()              loopCandidateTimer()       loopVerifyThread        visTimerFunc
  （每帧，前端同步）               (1 Hz, descriptor 阶段)   (后台消费 queue)        (0.5 Hz)
        │                                 │                          │                │
   累计 keyframe 触发           ILoopCandidateProvider        ILoopVerifier
   相邻 BetweenFactor           → push loopVerifyBuf           small_gicp align
   ISAM2.update                                                → Cauchy robust loop factor
        │                                                      └──  backpressure warn @ size>30
        └──▶ /corrected_odom  /corrected_path  /corrected_map
             /pose_stamped    TF(map→robot)    /loop_detection
```

**5 个独立锁**：
- `realtime_pose_mutex_` — 实时位姿（最高频）
- `keyframes_mutex_` — keyframe 容器
- `graph_mutex_` — GTSAM graph + init_esti
- `verify_buf_mutex_` — loopVerifyBuf 队列（SC-PGO 两阶段升级）
- `vis_mutex_` — 可视化缓冲

---

## 2. 数据结构（`PosePcd`）

```cpp
struct PosePcd {
  pcl::PointCloud<PointXYZI> pcd_;        // LiDAR frame（重要：不是 world frame）
  Eigen::Matrix4d pose_eig_;              // 原始前端 odom 姿态
  Eigen::Matrix4d pose_corrected_eig_;    // PGO 优化后的姿态
  double timestamp_;
  int idx_;                               // 单调递增
  bool processed_;                        // loop 线程的 claim flag
};
```

**关键细节**：
- `pcd_` 存 **LiDAR frame**（通过 `pcd_raw.transformed_by(pose.inverse())` 得到）。这样 keyframe 在回环 / save_map 时一律 `transformPcd(pcd_, pose_corrected_eig_)` 投到 world 即可。
- `pose_corrected_eig_` 初始等于 `pose_eig_`；loop 闭合后被 PGO 覆盖。

---

## 3. 三个主循环伪代码

### 3.1 odomPcdCallback（每前端帧）

```cpp
on (odom, pcd) sync:
  current_frame = PosePcd(odom, pcd, idx)     # 构造 + pcd → lidar frame

  // ---- 1. realtime pose = last_corrected × odom_delta ----
  lock(realtime_pose_mutex_):
    odom_delta_ = odom_delta_ * last_odom_tf.inv() * current.pose_eig_
    current.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_
    publish /pose_stamped + TF(map→robot)

  if !initialized:
    keyframes.push(current)
    graph.add(PriorFactor(0, pose, prior_noise))
    init_esti.insert(0, pose)
    initialized = true
    return

  // ---- 2. keyframe trigger（累计双阈值，SC-PGO 升级） ----
  dtf = diff(last_odom_tf, current.pose_eig_)       # Pose6D 增量
  trans_accum += |dtf.translation|                  # 累计平移
  rot_accum   += |dtf.roll + dtf.pitch + dtf.yaw|   # 累计旋转（朴素和）

  if trans_accum < keyframe_meter_gap &&
     rot_accum   < keyframe_rad_gap:
    return                                          # 未触发
  trans_accum = 0; rot_accum = 0                    # 触发后重置

  // ---- 3. add keyframe & between factor ----
  lock(keyframes_mutex_): keyframes.push(current)
  lock(graph_mutex_):
    from = keyframes[idx-1].pose_corrected
    to   = current.pose_corrected
    graph.add(BetweenFactor(idx-1, idx, from.between(to), odom_noise))
    init_esti.insert(idx, to)

  // ---- 4. ISAM2 update ----
  lock(graph_mutex_):
    isam.update(graph, init_esti)
    isam.update()
    if loop_added_flag_:
      isam.update() × 3    # from LIO-SAM issue #5
    graph.resize(0); init_esti.clear()

  // ---- 5. apply corrected result ----
  lock(realtime_pose_mutex_):
    corrected_esti = isam.calculateEstimate()
    last_corrected_pose_ = corrected_esti.back()
    odom_delta_ = Identity
  if loop_added_flag_:
    lock(keyframes_mutex_):
      for i in 0..N: keyframes[i].pose_corrected = corrected_esti[i]
    loop_added_flag_ = false
```

### 3.2 loopCandidateTimer（1 Hz 后台，descriptor stage）

```cpp
timer @ 1 Hz:
  latest = keyframes.back()
  if latest.processed_: return
  latest.processed_ = true

  # C-1: radius-based provider (FAST-LIO-SAM 路线)
  # C-2: 这里会替换成 ILoopCandidateProvider 插件（SOLiD / STD / SC）
  closest_idx = fetchClosestKeyframeIdx(latest, keyframes)
  # 条件: dist < loop_radius AND (t_now - t_closest) > timediff_thr
  if closest_idx < 0: return

  # backpressure（SC-PGO 升级）
  if loopVerifyBuf.size() > 30:
    WARN("loop verifier 跟不上，降低 provider 频率或提高 reject")

  lock(verify_buf_mutex_):
    loopVerifyBuf.push({latest.idx, closest_idx})
```

### 3.3 loopVerifyThread（后台独立，geometry stage，SC-PGO 两阶段升级）

```cpp
thread loop-verify:
  while running:
    if loopVerifyBuf.empty(): sleep(10ms); continue
    lock(verify_buf_mutex_): {prev_idx, curr_idx} = loopVerifyBuf.pop()

    (src_submap, dst_submap) = setSrcAndDstCloud(
        keyframes, curr_idx, prev_idx,
        submap_range=5, voxel_res=0.3)
    # 取 ±N keyframes, transform 到 world, voxelize

    reg = smallGicpVerifier(src_submap, dst_submap, init_T=Identity)
    # C-1: hasConverged + fitness < threshold
    # C-2: 用 result.H.inverse() 作 covariance

    if !reg.is_valid: continue

    pose_from  = reg.T * keyframes[curr_idx].pose_corrected
    pose_to    = keyframes[prev_idx].pose_corrected

    # Cauchy robust loop noise（SC-PGO 升级，替代 fitness 各向同性）
    base_noise = Diagonal::Variances(0.5 × 6)          # 固定
    loop_noise = Robust::Create(
                   mEstimator::Cauchy::Create(1),
                   base_noise)

    lock(graph_mutex_):
      graph.add(BetweenFactor(curr_idx, prev_idx,
                              pose_from.between(pose_to), loop_noise))
    loop_added_flag_ = true       # 触发下一帧的 4× update
    loop_added_flag_vis_ = true
```

### 3.4 visTimerFunc（0.5 Hz）

```cpp
timer @ 0.5 Hz:
  if loop_added_flag_vis_:
    # 从 ISAM2 拷贝最新 estimate, 刷新 corrected_odoms / corrected_path / loop markers
  publish ori_odom / ori_path / corrected_odom / corrected_path
  # global map: 仅当 subscriber 数 > 0 时生成, 避免无效开销
```

---

## 4. GTSAM 调用要点（合并 SC-PGO 升级）

| 项 | 值 | 来源 |
|---|---|---|
| Optimizer | `ISAM2` (not LevenbergMarquardt) | FAST-LIO-SAM |
| `relinearizeThreshold` | 0.01 | FAST-LIO-SAM |
| `relinearizeSkip` | 1 | FAST-LIO-SAM |
| Prior noise | `diag(1e-12 × 6)` 硬约束 | SC-PGO（替代 FAST-LIO-SAM 的 1e-4/1e-2） |
| Odom factor noise | `diag(1e-6 rot², 1e-4 m²)` **区分 rot/trans** | SC-PGO |
| Loop factor noise | **`Robust::Cauchy(1) + Diagonal(0.5 × 6)`** | SC-PGO |
| 有 loop 后 `update()` 次数 | `1 + 1 + 3 = 5` | FAST-LIO-SAM (LIO-SAM #5) |
| GPS altitude noise（可选，Phase D+） | `Robust::Cauchy(1) + Diag(1e9, 1e9, 250)` | SC-PGO |

**关键技巧变更**：
- 原 FAST-LIO-SAM：loop noise = `Diagonal(score × 6)`，让 ICP 差的回环被削权。简单但无 outlier robustness。
- 升级后（SC-PGO）：loop noise = `Cauchy(1) + Diagonal(0.5)`。**Cauchy kernel 自动对 outlier 削权**，不依赖 score 本身的数值精度。
- 对我们的含义：C-1 先走 Cauchy；C-2 升级到 `result.H.inverse()`（来自 small_gicp 原生 API）作 Gaussian covariance，Cauchy 作外层包装。

**为什么 prior noise 从 1e-4 改到 1e-12**：第一帧的位姿本来就是定义原点（identity），属于 "人造真值"，应硬约束。1e-4 那档是给**非 prior 的节点**用的。这个改动纯属修正 FAST-LIO-SAM 的一个小 bug。

**odom noise 变严的理由**：FAST-LIO2 IESKF 前端相邻帧精度 < 1mm / 0.1mrad，SC-PGO 的 `1e-6 rot² = (1mrad)²` / `1e-4 trans² = (1cm)²` 更贴合实际。原值 `1e-4 rot² / 1e-2 trans²` 是给 LOAM 前端的。

---

## 5. 实时 pose 补偿（map correction 的核心公式）

```
last_corrected_pose  = PGO 最近一次优化后的最后一个关键帧位姿
odom_delta           = 从 last_keyframe 到当前帧的相对 odom（每次 PGO 重置为 Identity）

realtime_pose        = last_corrected_pose × odom_delta
realtime_pose        → 同时广播 TF(map→robot) 和 /pose_stamped
```

**不回写前端**。前端 `/Odometry` 保持原样，FAST-LIO2 状态不被污染。回环的影响只通过这个 TF + 新的 `corrected_*` topic 下发给下游。

---

## 6. 回环细节（我们 C-1 要改的地方）

| FAST-LIO-SAM 原实现 | 我们 C-1 建议 |
|---|---|
| pcl::IterativeClosestPoint | **small_gicp**（PCL drop-in 模式，见 `LOOP_VERIFIER_SELECTION.md`） |
| max_corr_dist = radius × 1.5 | 同，从 yaml 外化 |
| submap_range=5 ± keyframes | 从 yaml 外化。温室小到 3，林地大到 7-10 |
| loop_detection_radius=15m | 温室缩到 5m，林地可保持 15m |
| loop_detection_timediff=10s | 保持 |
| icp_score_threshold=0.3 | 保持作 fallback；GICP 用 inlier_ratio 或 chi² |
| 只有 radius-based candidate | C-1 暂时保留；C-2 加 `ILoopCandidateProvider`（SOLiD） |
| Keyframe 触发 = 单次 dist | **累计双阈值（SC-PGO 升级）**：`trans_acc > keyframe_meter_gap OR rot_acc > keyframe_rad_gap` |
| Keyframe 阈值 | 温室 `meter=0.3m, rad=0.1 rad`；林地 `meter=1.5m, rad=0.2 rad` |
| 单线程 loop timer（descriptor+ICP 同步） | **两阶段 pipeline（SC-PGO 升级）**：loopCandidateTimer → queue → loopVerifyThread |
| 无 backpressure | **loopVerifyBuf.size() > 30 WARN（SC-PGO 升级）** |
| Backend 订阅 topic = hardcode | **yaml 外化（SC-PGO 哲学）**：`slam_core/backend/topics/{odom,cloud,gps}` |

---

## 7. 对应到我们 slam_core 的落地

FAST-LIO-SAM 是单文件 monolith（`FastLioSam` 一个类吃所有状态）。我们 **不要照抄单类**，按深度报告的插件化建议抽 5 个接口：

```
src/FAST_LIO_SUPER/
  include/slam/graph/
    keyframe_store.hpp         # IKeyframeManager + Keyframe 数据结构
    pose_graph.hpp             # IPoseGraph (iSAM2 封装)
    loop_candidate_provider.hpp # 插件接口；C-1 默认 RadiusProvider
    loop_verifier.hpp          # 插件接口；C-1 默认 SmallGicpVerifier
    map_correction_bridge.hpp  # last_corrected × delta 公式的载体
  src/slam/graph/
    keyframe_store.cpp
    pose_graph_isam2.cpp
    loop_candidate_radius.cpp
    loop_verifier_small_gicp.cpp
    map_correction_bridge.cpp
  nodes/
    fast_lio_super_backend_node.cpp    # 新 ROS node，sidecar 部署
```

**C-1 暂不做**：
- SOLiD / STD / 任何 descriptor（C-2 才做）
- ForestLPR / TreeLoc（C-2）
- BA（Phase D）
- Relocalizer / MapIO（C-3）

---

## 8. 对应代码量估计（我们自己写）

| 模块 | 预估行数 | 参考 |
|---|---:|---|
| Keyframe 数据结构 + store | 150 | pose_pcd.hpp + 部分 fast_lio_sam.cpp |
| RadiusLoopCandidate | 100 | fetchClosestKeyframeIdx |
| SmallGicpLoopVerifier | 200 | loop_closure.cpp + small_gicp API |
| PoseGraphIsam2 | 300 | 包含 odom callback 那 5 段 |
| MapCorrectionBridge | 150 | realtime_pose 三段 |
| Backend node（ROS 胶水） | 150 | main + cb + timer |
| utilities（Eigen↔GTSAM） | 100 | utilities.hpp |
| **合计** | **~1150** | 与 FAST-LIO-SAM 1056 行对齐 |

外加 docker 镜像加 GTSAM 4.0.3 源码编译层（30min 一次性）。

---

## 9. FAST-LIO-SAM + SC-PGO 都没做的、我们应该做的

1. **配置外化**：原两者都部分 hardcode，我们全量 `slam_core/backend/` 子段 yaml。
2. **温室 vs 林地 profile**：keyframe 阈值 / loop_radius / timediff / submap_range 做成 `scene_profile`。
3. **锁粒度**：已对齐到 5 个 mutex（含 verify_buf），细粒度够用。
4. **按距离累积 submap**：C-1 先保持按帧数（±N keyframes），C-2 评估改按距离（保证稀疏点云场景下的稳健性）。
5. **candidate provider 插件化**：C-1 默认 RadiusProvider；C-2 换 SOLiD/STD/SC。
6. **verifier 插件化**：C-1 small_gicp PCL drop-in；C-2 可叠 Quatro 粗配（参考 FAST-LIO-SAM-QN）或升级到 native API 拿 H。
7. **ISAM2 参数外化**：`relinearizeThreshold / relinearizeSkip / update_count_after_loop` 从 yaml 读。
8. **TF 广播 map→odom vs map→robot**：原版 map→robot。我们建议 **map→odom**（更标准，下游 `greenhouse_nav` TF 树不破坏），由 `MapCorrectionBridge` 负责维护。
9. **condition_variable 替代忙等待**：SC-PGO 用 `while(1) + buf check`，生产代码应换成 `std::condition_variable`（避免 loop 线程空转消耗 CPU）。

---

## 10. 建议 C-1 开工顺序

1. docker 镜像加 GTSAM 4.0.3 层（30 min）
2. 写 `include/slam/graph/*.hpp` 接口头（1 day）
3. 写 `keyframe_store.cpp` + `map_correction_bridge.cpp`（0.5 day）
4. 写 `pose_graph_isam2.cpp`（1.5 day，含 `odomPcdCallback` 对应的 5 段）
5. 写 `loop_candidate_radius.cpp` + `loop_verifier_small_gicp.cpp`（1 day）
6. 写 backend_node.cpp ROS 胶水（0.5 day）
7. 把 small_gicp 作为 catkin 依赖加进来（或 vendored header-only）
8. 改 launch：`fast_lio_super_mapping.launch` 加一个 `<node pkg="fast_lio_super" type="fast_lio_super_backend_node"/>`
9. 编译 + 回放 bag 测试（回归 FAST-LIO2 前端不受影响）

**预估总耗时**：5-7 个工作日（不含测试/调参）

---

## 11. 风险与注意

1. **GTSAM 4.0.3 vs 4.2+ API 不兼容** — docker 锁定 tag。
2. **submap transform 用 corrected_pose**，意味着 PGO 越准，回环越好；但初始阶段尚未收敛时 corrected = original，问题不大。
3. **Loop noise Cauchy(1) 的经验值**：作者 gisbi-kim 注释 *"replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good"*。C-1 用 Cauchy；若林地实测 outlier 多则换 DCS。
4. **small_gicp GICP 的 fitness score 含义**与 pcl::ICP 不同（plane-to-plane vs point-to-point），阈值需重校准。C-1 先保守设 0.3，实测再调。
5. **realtime pose 公式 `last_corrected × odom_delta`** 在 PGO 优化耗时 > 帧间隔时可能出现老数据被新 delta 乘的问题；原版用锁串行化，我们照做。
6. **两阶段 pipeline 延迟**：descriptor 1Hz → queue → ICP 可能积压。我们用 backpressure warn，但实际是否需要**丢弃旧 candidate**（仅保留最新 N 个）要实测。
7. **累计 rot 阈值的模型简陋**：SC-PGO 直接 `rot_acc += |roll+pitch+yaw|`，小旋转快速累加会误触发。C-1 照抄；若问题明显，C-2 换成 `acos(trace(R_delta)/2 - ...)` 真实角度。
8. **TF `map→odom` 和原 FAST-LIO-SAM 的 `map→robot` 不一致** —— launch 文件和下游 rviz/nav 栈 TF 期望需要同步修改。我们 greenhouse_nav 的 TF 结构要先捋清楚再定方案。

---

## 12. 参考文件行号速查

### FAST-LIO-SAM（主骨架）
- `fast_lio_sam.cpp:64-179` — odomPcdCallback 主循环
- `fast_lio_sam.cpp:181-229` — loopTimerFunc
- `fast_lio_sam.cpp:475-478` — checkIfKeyframe（两行）
- `fast_lio_sam.cpp:30-35` — ISAM2 初始化
- `loop_closure.cpp:18-40` — fetchClosestKeyframeIdx（候选筛选）
- `loop_closure.cpp:42-67` — setSrcAndDstCloud（子图积累）
- `loop_closure.cpp:69-92` — icpAlignment
- `pose_pcd.hpp:21-43` — PosePcd 构造

### SC-PGO（融合升级来源）
- `laserPosegraphOptimization.cpp:214-240` — initNoises（Cauchy robust + 各向异性 GPS）
- `laserPosegraphOptimization.cpp:480-635` — process_pg（累计 keyframe 触发）
- `laserPosegraphOptimization.cpp:637-655` — performSCLoopClosure + process_lcd（descriptor stage）
- `laserPosegraphOptimization.cpp:668-699` — process_icp（geometry stage + backpressure）
- `laserPosegraphOptimization.cpp:713-729` — process_isam（独立 ISAM update 线程）
- `laserPosegraphOptimization.cpp:820-826` — 6 std::thread 创建
- `Scancontext.h:60-110` — 核心 API（`makeAndSaveScancontextAndKeys` / `detectLoopClosureID`）

---

**后续**：
- **F** — 读 SOLiD C++ 269 行，评估 Mid360 descriptor 接入（C-2 铺垫）
- **D** — 直接动手 C-1：docker 加 GTSAM → 试编 small_gicp → 写 `include/slam/graph/*.hpp`
- **G** — docker 预编译验证（最稳路径，仅验证环境）
