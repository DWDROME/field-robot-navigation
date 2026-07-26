# SC-PGO（gisbi-kim/FAST_LIO_SLAM）对比分析

**来源**：`~/refs/fast_lio_super/FAST_LIO_SLAM/SC-PGO/`
**License 状态**：SC-PGO/package.xml 写 `BSD`（继承 A-LOAM）；根仓库无 LICENSE；ScanContext 文件头无独立声明（上游 `irapkaist/scancontext` 原本是 MIT）。**保守按"只作图纸"处理**，并库前必须再确认。
**日期**：2026-04-18
**对比对象**：FAST-LIO-SAM (engcang) — 见 `C1_BACKEND_SKELETON.md`

---

## 0. 核心结论

SC-PGO 在**两件事**上显著优于 FAST-LIO-SAM（engcang），其它方面基本等价：

1. **真正的两阶段 loop pipeline**：`process_lcd`（descriptor 1Hz）→ queue → `process_icp`（后台 ICP）→ `process_isam`。与深度报告 v2 的推荐完全对齐。
2. **更成熟的 noise 建模**：Cauchy robust kernel + 各向异性（odom 分 rot/trans，GPS 只信 Z）。比 FAST-LIO-SAM 的 "fitness 各向同性" 更专业。

其它项（通用 topic 接口、keyframe 累计触发、独立节点）属于**小优化**，可吸收。

---

## 1. 两张骨架对比

| 维度 | FAST-LIO-SAM (engcang) | SC-PGO (gisbi-kim) |
|---|---|---|
| 代码结构 | 单类 `FastLioSam` 1056 行 | 主文件 `laserPosegraphOptimization.cpp` 831 行，+ Scancontext ~550 行 |
| 线程模型 | 4-thread async spinner + 2 timer | **6 个显式 `std::thread`** |
| Loop detection 方式 | radius + time（几何 only） | **ScanContext descriptor** + ICP verify（两阶段） |
| Loop 判收 | fitness score 阈值 | ICP + Cauchy robust kernel |
| Loop factor noise | 各向同性 fitness score | **Cauchy(1) + diag(0.5)** 固定鲁棒模型 |
| Odom factor noise | `diag(1e-4 rot², 1e-2 m²)` | `diag(1e-6 rot², 1e-4 m²)` —— **区分 rot/trans** |
| Keyframe 触发 | 单次 `dist > thr` | **累计** `trans_acc > 2m OR rot_acc > 0.2 rad` |
| GPS altitude | ❌ | ✅ `robustGPSNoise` 各向异性（xy=1e9, z=250） |
| Map correction 回写前端 | ❌ | ❌ |
| 通用前端 | 特化 FAST-LIO2 topic | **任意 odom + cloud topic**（README 强调） |
| 总代码（去 LOAM 残留） | 1056 行 | ~1500 行 |

---

## 2. SC-PGO 6 线程架构详解

```
ROS callback 线程（主）
  ├─ laserOdometryHandler     → odometryBuf
  ├─ laserCloudFullResHandler → fullResBuf
  └─ gpsHandler               → gpsBuf
                                     ↓
        ┌────────────────────────────┴──────────────────────────────┐
        │                                                            │
   process_pg（无 sleep，忙循环）     process_lcd（1 Hz rate.sleep）
   · 从 buf 取 odom+cloud              · scManager.detectLoopClosureID()
   · 累计 trans/rot → keyframe 判定      → {loop_idx, yaw_diff}
   · make SC descriptor                · 成功就 push scLoopICPBuf
   · 添加 odomFactor（+ GPS）           
   · 初估 / 维护 keyframePoses           
                                     ↓                              ↓
                              process_icp（无 sleep，消费 queue）
                              · 从 scLoopICPBuf 取 (prev, curr)
                              · submap ±N keyframes，pcl::ICP
                              · 成功 → 添加 loopFactor（Cauchy robust）
                                     ↓
                              process_isam（无 sleep）
                              · ISAM2.update(graph, init_esti)
                              · 拿 corrected_poses
                                     ↓
             process_viz_path（10 Hz）      process_viz_map（0.1 Hz，低频）
             · 刷新 /aft_pgo_path            · 全地图重拼装
             · 高频发布，低开销              · 订阅者 > 0 才发
```

**关键设计**：
- `process_lcd` 用 `rate.sleep()` 限速（1Hz）；`process_pg / icp / isam` 用 `while(1)+buf check` 忙等待。
- `scLoopICPBuf.size() > 30` warning — 明确的 **backpressure 机制**。
- `process_isam` 独立线程后作者备注：*"if you want to call less isam2 run...uncomment this"* — 也就是 ISAM update 完全可异步化。

**对比 FAST-LIO-SAM**：后者在 `odomPcdCallback` 里同步做 ISAM update，高频 keyframe 会阻塞回调。SC-PGO 的分层更扛实时压力。

---

## 3. 两阶段 loop pipeline（v2 推荐的模式）

### 3.1 Stage 1 — descriptor 匹配

```cpp
// process_lcd @ 1Hz
void performSCLoopClosure(void) {
  if (keyframePoses.size() < scManager.NUM_EXCLUDE_RECENT) return;
  auto [loop_idx, yaw_diff] = scManager.detectLoopClosureID();
  if (loop_idx != -1) {
    scLoopICPBuf.push({loop_idx, curr_idx});   // descriptor-first
  }
}
```

**ScanContext API 干净度**（API 级借鉴价值）：
- `makeAndSaveScancontextAndKeys(cloud)` — 构建 descriptor
- `detectLoopClosureID() → {idx, yaw_diff}` — 一键查询
- 内部：polar grid + ring key + sector key + ring-level distance

### 3.2 Stage 2 — ICP 验证

```cpp
// process_icp 独立线程
while (!scLoopICPBuf.empty()) {
  auto [prev_idx, curr_idx] = scLoopICPBuf.front(); scLoopICPBuf.pop();
  auto near_keyframes = accumulateSubmap(prev_idx, ±25);
  pcl::ICP.setInput(near_keyframes, query);
  if (icp.hasConverged() && icp.getFitnessScore() < threshold) {
    graph.add(BetweenFactor(prev_idx, curr_idx, T, robustLoopNoise));
  }
}
```

**与 FAST-LIO-SAM 的区别**：FAST-LIO-SAM 用 `radius < 15m && time > 10s` 筛一个 candidate；SC-PGO 用 descriptor 检索 N 个候选再交给 ICP 逐一验证。在**大回环 / 反向访问 / 视觉相似环境**下，descriptor 命中率明显高于纯几何半径。

**对我们的含义**：
- C-1 图纸上**保留 radius-based 作为基线**（FAST-LIO-SAM 路线）
- C-2 的 `ILoopCandidateProvider` 直接按 SC-PGO 的 "push to queue" 模式实现，后面插 SOLiD 时接口不用改

---

## 4. Noise 建模（值得直接抄的 3 条）

### 4.1 Odom factor 区分 rot/trans（推荐抄）

```cpp
// FAST-LIO-SAM
odomNoise = Diagonal({1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2});  // rad² rad² rad² m² m² m²

// SC-PGO（更严）
odomNoise = Diagonal({1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4});  // FAST-LIO2 前端很稳，相邻 keyframe 间的 odom 误差应该很小
```

**对我们**：FAST-LIO2 的相邻帧 odom 精度很高（~0.001m / 1mrad），SC-PGO 的 1e-6 rot / 1e-4 trans 更符合实际。**推荐用 SC-PGO 值作为 C-1 默认**。

### 4.2 Loop factor 用 Cauchy robust kernel（推荐抄）

```cpp
robustLoopNoise = Robust::Create(
    mEstimator::Cauchy::Create(1),              // Cauchy kernel，outlier 自动削权
    Diagonal::Variances(0.5 × 6));              // 固定 0.5，不再依赖 fitness
```

作者注释：*"replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good."*

**对我们**：深度报告 v2 推荐 "H.inverse() 或 inlier-ratio 作 loop noise"，属于更精细路线。SC-PGO 的 Cauchy 是**低成本 fallback**，C-1 完全可用；C-2 升级到 small_gicp 的 H.inverse() 时，再把 Cauchy 作为 second-line defense 外包一层。

### 4.3 GPS altitude robust factor（C-1 不做，C-2+ 可选）

```cpp
robustGPSNoise = Robust::Create(
    mEstimator::Cauchy::Create(1),
    Diagonal::Variances({1e9, 1e9, 250}));      // xy 完全不信，z = 250
```

**意义**：LOAM 家族（含 FAST-LIO2）z 轴"飘"是已知问题。GPS 给的 altitude 即使误差 10m 也比 LIO 累计 z 漂移好。xy 设 1e9 等于完全忽略 GPS 水平分量（通常 GPS 水平精度 < LIO 水平精度）。

**对我们**：
- 温室室内：无 GPS，不需要
- 林地室外：可选加 GPS altitude factor，专治 z 飘
- C-1 不做，**在 `slam_core/backend/` 的 yaml schema 里预留 `enable_gps_altitude: false`**，C-2 或 Phase D 再接

---

## 5. Keyframe 触发方式（轻微优化，值得抄）

```cpp
// FAST-LIO-SAM（单次距离）
return dist(current, last_keyframe) > threshold;   // 默认 1.0m

// SC-PGO（累计 + 双阈值）
translationAccumulated += |delta_trans|;
rotationAccumulated += |delta_roll + delta_pitch + delta_yaw|;
if (trans_acc > keyframeMeterGap || rot_acc > keyframeRadGap) {
    is_keyframe = true;
    trans_acc = 0; rot_acc = 0;
}
```

**差异**：
- 单次距离会在机器人"蜿蜒前进"时漏 keyframe（每帧走 0.3m，但 5 帧累计已走 1.5m）
- 累计阈值保证 **等间距 keyframe + 拐弯时额外触发**
- SC-PGO 默认 `keyframeMeterGap = 2.0m`，`keyframeRadGap = 0.2 rad (~11°)`

**对我们**：温室小尺度场景，推荐 `meter=0.3m, rad=0.1 rad`；林地 `meter=1.5m, rad=0.2 rad`。做成 `scene_profile` 参数。

---

## 6. 通用前端设计（哲学吸收，不涉及代码）

SC-PGO README 强调：
> **The only difference from A-LOAM is the addition of `laserPosegraphOptimization.cpp`.** That is, our implementation is **generic to any front-end odometry methods**.

对应 topic 接口：
- `/aft_mapped_to_init`（nav_msgs/Odometry）
- `/velodyne_cloud_registered_local`（sensor_msgs/PointCloud2）
- `/gps/fix`（可选）

**意义**：SC-PGO **明确把自己定位成"后端插件"而非"某一套 SLAM 的后端"**。

**对我们**：C-1 backend_node 的订阅 topic 应该**外化到 yaml**（不 hardcode FAST-LIO2 的 `/Odometry`），让这个 node 未来可以接 Super-LIO、FAST-LIVO2、任何前端。这个原则吸收进 `C1_BACKEND_SKELETON.md` 第 11 节。

---

## 7. SC-PGO 的负面信号

1. **目录有 LOAM 残留**：`scanRegistration.cpp` / `laserOdometry.cpp` / `laserMapping.cpp`（617+952 行）是 A-LOAM 原版代码未删。FAST_LIO_SLAM 实际运行不需要。C-1 借鉴时要**只看 `laserPosegraphOptimization.cpp`**。
2. **6 线程调试复杂度**：比 3 timer 模型高。C-1 首版**建议先用 FAST-LIO-SAM 的 3 timer**，等架构稳了再渐进细分到 6 线程。
3. **License 模糊**：
   - 根仓库：无 LICENSE
   - SC-PGO/package.xml：写 BSD（适用于 A-LOAM 继承部分）
   - Scancontext.h/cpp：无文件头 license 标注
   - `laserPosegraphOptimization.cpp`：无文件头 license 标注
   - **我们的立场**：**保守作图纸**。如要引入 ScanContext 代码到项目，应从 `irapkaist/scancontext` 原始仓库 clone（明确 MIT）。
4. **OpenCV 依赖**：ScanContext.h 包含 `opencv2/opencv.hpp`，cv_bridge。我们 `fast_lio_super` 当前**不依赖 OpenCV**，引入 SC 会多一层依赖（noetic apt 有，代价小）。
5. **6 个 `std::thread` 全程 while(1) 无 cpu yield**：忙等待消耗 CPU。生产级代码应该用 `condition_variable` 或 `std::queue::wait`。是个可吐槽点。

---

## 8. 借鉴建议整理

**立即吸收到 C-1**（C1_BACKEND_SKELETON.md 更新）：
1. Odom factor noise：`rot=1e-6, trans=1e-4`（区分 rot/trans）
2. Loop factor noise：Cauchy(1) + Diagonal(0.5)（outlier 鲁棒）
3. Keyframe 累计触发（双阈值，meter + rad）
4. ScaneSubscribe topic 外化到 yaml（通用前端设计）
5. Backpressure 机制：loop candidate queue size 检查

**C-2 吸收**：
6. 两阶段 pipeline 明确：`ILoopCandidateProvider` → queue → `ILoopVerifier`
7. ScanContext 作为 candidate provider 的其中一个插件（与 SOLiD 并列）

**Phase D 吸收**：
8. GPS altitude robust factor（林地室外用）
9. 6 线程细分架构（生产级细粒度）

**不吸收**：
- SC-PGO 的 LOAM 残留代码（无关）
- 忙等待 `while(1)` 模式（换 condition_variable）
- Scancontext.cpp 直接抄（去 `irapkaist/scancontext` 原仓库）

---

## 9. 两图纸合并后的 C-1 骨架（增量更新）

相对 `C1_BACKEND_SKELETON.md` 第 4 节（GTSAM 调用要点），新增：

| 项 | 原值 | 更新后 |
|---|---|---|
| Odom factor noise | `diag(1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2)` | `diag(1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4)` ← SC-PGO |
| Loop factor noise | `diag(score × 6)` 各向同性 | **`Robust::Cauchy(1) + diag(0.5 × 6)`** ← SC-PGO |
| Keyframe 触发 | 单次 `dist > 1m` | **累计** `trans > 0.3m (温室) / 1m (林地) OR rot > 0.1 rad` ← SC-PGO |
| Backend 订阅 topic | hardcode `/Odometry` | yaml 外化 `slam_core/backend/topics/{odom,cloud}` ← SC-PGO 哲学 |
| Loop candidate queue | 无 | 加 backpressure warn@ size>30 ← SC-PGO |

---

## 10. 两条 sidecar 路线的最终判断

| | FAST-LIO-SAM | SC-PGO |
|---|---|---|
| 可直接作 C-1 骨架？ | ✅ 代码最干净 | ❌ LOAM 残留多，需裁剪 |
| noise 建模成熟？ | 粗糙 | ✅ |
| 两阶段 loop pipeline？ | ❌（单阶段几何） | ✅ |
| 通用前端哲学？ | 偏特化 | ✅ |
| License 风险？ | CC-BY-NC-SA 明确 | BSD 标着但根无 LICENSE，模糊 |

**结论**：**C-1 主骨架继续按 FAST-LIO-SAM 走**（干净易读），**关键参数和 noise 建模吸收 SC-PGO**。两者合并成一份内部图纸。

---

## 11. 下一步选项

- **D** — 直接动手 C-1：docker 加 GTSAM → 试编 small_gicp（PCL 1.10 兼容） → 写接口头
- **F** — 读 SOLiD CPU C++ 实现（269 行）评估接入（Mid360 候选 descriptor）
- **G** — 仅做 docker 预编译验证（GTSAM + small_gicp 环境稳定性）
- **合并** — 把本文第 8/9 节的更新落到 `C1_BACKEND_SKELETON.md`（保持单一真源）

回字母/组合。推荐 **合并 + G**（最稳路径）或 **合并 + D**（最快推进）。
