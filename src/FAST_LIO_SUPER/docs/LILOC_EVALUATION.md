# LiLoc 整包引入评估

**来源**：`~/refs/fast_lio_super/LiLoc/`（MIT 许可）
**论文**：Fang et al., *LiLoc: Lifelong Localization using Adaptive Submap Joining and Egocentric Factor Graph*, ICRA 2025
**调研日期**：2026-04-18
**结论一句话**：**不建议整包引入**。只借 3 个关键模块（~1350 行）到 C-3。

---

## 0. 最关键发现

LiLoc **不是独立 relocalizer**，而是 **LIO-SAM 的完整 fork**，包含前端特征提取、IMU 预积分、后端、定位四层。整包引入等于**把我们前端换成 LIO-SAM 风格**，与 Phase A/B 的方向相反。

| 文件 | 行数 | 来源 |
|---|---:|---|
| `src/mapOptmization.cpp` | 1,634 | LIO-SAM fork + 作者修改 |
| `src/imageProjection.cpp` | 695 | LIO-SAM 原版（LOAM 特征提取） |
| `src/imuPreintegration.cpp` | 463 | LIO-SAM 原版 |
| `include/ikd-Tree/` | 1,714 | FAST-LIO 的 ikd-tree |
| **`include/egoOptimization/`** | **650** | **作者真正创新**（anchor EFG） |
| **`include/dataManager/`** | **691** | **作者真正创新**（session + submap） |
| `include/utility.h` + `math_tools.h` | 944 | LIO-SAM 风格 ParamServer |

**真正的创新只占 18%**（1341 / 7288 行）。其余 82% 是继承自 LIO-SAM 的基础设施，我们已经有 FAST-LIO2 来做同样的事。

---

## 1. 依赖链

```
package.xml 声明:
  - roscpp / tf / cv_bridge / pcl_ros（标准）
  - ndt_omp（ros-noetic-ndt-omp，apt 有）
  - GTSAM 4.0.3+（源码编译，docker 加一层）
  - livox_ros_driver（v1 API，注意：我们用 v2）  ❗
  - OpenCV 4.0+

说 README 明确要求:
  - better_fastlio2（GPL-2.0，与我们 GPLv3 不兼容）用于 pose initialization  ❗
```

**两个红灯**：
1. `livox_ros_driver`（v1）与我们 `livox_ros_driver2`（v2）**不兼容** — topic、msg、驱动都是独立代码。
2. 自动重定位初值**未完成**：README 自述 *"code of pose initialization is under reconstruction, use 2D pose estimation in RViz or refer to better_fastlio2"*。而 better_fastlio2 是 GPL-2.0 —— 我们不能用。

---

## 2. License 细节

| 项 | 声明 |
|---|---|
| 根 `LICENSE` 文件 | **MIT** (Copyright 2023 YJZ_Lucky_Boy) |
| `package.xml` 的 `<license>` | `TODO`（作者没填） |
| README 提到的 acknowledgements | LIO-SAM（MIT）、liorf、Block-Map-Based-Localization |

以根 LICENSE 为准，MIT，与我们 GPLv3 兼容。**技术上可并库**。但基于第 1 节的依赖问题，"整包" 没价值。

---

## 3. 真正有价值的 3 个模块

### 3.1 `BetweenFactorWithAnchoring.h`（164 行）

**功能**：自定义 GTSAM `NoiseModelFactor4`，绑 4 个变量（两 session 的 pose + 两 anchor），实现 Kim 2010 multi-session ISAM anchor node。

**核心公式**：
```cpp
hx1   = compose(anchor_p1, p1)
hx2   = compose(anchor_p2, p2)
error = Local(measured, Between(hx1, hx2))
```

**借鉴价值**：直接抄。GPL-compatible MIT，单文件零依赖（只依赖 GTSAM），是 multi-session 功能的基石。

### 3.2 `egoOptimization/anchorOptimize.hpp`（486 行）

**功能**：`AnchorOptimization` 类，管理 prior session + current session 的联合优化。

**关键字段**：
```cpp
shared_ptr<Session> priorSession_;          // 从磁盘加载的历史会话
pcl::Registration<PointType>::Ptr registration_;  // NDT-omp 或 GICP-omp
gtsam::ISAM2* isam_;
gtsam::NonlinearFactorGraph gtSAMgraph_;

// 4 档噪声（值得借）
priorNoise_  : diag(1e-12)   // 固化 prior session
odomNoise_   : diag(1e-6)
matchNoise_  : 待看剩余代码
largeNoise_  : 待看
robustNoise_ : Huber or 其他 robust kernel
```

**借鉴价值**：C-3 的 `IRelocalizer` 实现骨架直接参考。可以不抄代码，抄结构。

### 3.3 `dataManager/dataLoader.hpp`（394 行）+ `dataSaver.hpp`（297 行）

**功能**：
- `Session` 类：管理一个会话的 `KeyPoses6D_`、`globalMap_`、`usingSubMap_`
- 关键参数：`submap_size = 10`（一个 submap 多少个 keyframe）、`search_gap = 10`
- 磁盘格式：`transformations.pcd` + `globalMap.pcd` + `singlesession_posegraph.g2o` + per-keyframe 子目录

**借鉴价值**：C-3 的 MapIO 格式直接抄。G2O 格式 posegraph + PCD keyframe 是标准选择，以后多工具能读。

---

## 4. LiLoc 没解决、我们必须自己做的

1. **自动重定位初值**：LiLoc README 明确写 *"under reconstruction"*。我们 C-3 需要自己接 SOLiD（已 clone，MIT）+ small_gicp 做几何验证。
2. **Livox Mid360 v2 驱动集成**：LiLoc 还在 v1，我们要自己连 `livox_ros_driver2` 的 topic。
3. **与 FAST-LIO2 前端的衔接**：LiLoc 是 mapOptmization-centric，我们是 FAST-LIO2-centric。关键帧来源、点云 frame（body vs world）都不一样。
4. **Scene profile（温室 vs 林地）**：LiLoc 的参数是 NCLT / M2DGR 的城市/建筑场景调的，我们要重调。

---

## 5. 与我们 C-1 图纸的关系

| Phase | 要做的 | 参考对象 |
|---|---|---|
| C-1 | keyframe + odom factor + 本session loop | **FAST-LIO-SAM**（C1_BACKEND_SKELETON.md） |
| C-2 | ILoopCandidateProvider 插件化 | **SOLiD** / STD |
| C-3 | **Relocalizer + 多 session + MapIO** | **LiLoc**（本文档） + small_gicp |
| D  | persistent voxel sidecar | Bonxai |

**关键判断**：C-1 里**不引入** LiLoc 的任何代码。C-3 开工时再回来借 3 个模块。

---

## 6. C-3 时的借鉴清单（估工程量）

| 项 | 借鉴自 | 我们代码量 | 备注 |
|---|---|---:|---|
| `BetweenFactorWithAnchoring` | 直接抄 `.h` | 164 | 单文件，零侵入 |
| Session 抽象 + disk format | `dataLoader/Saver` 思路 | ~500 | 不抄实现，重写以适配 FAST-LIO2 keyframe 格式 |
| AnchorOptimization / EFG | `anchorOptimize.hpp` 思路 | ~400 | 重写，集成我们的 `IPoseGraph` |
| 自动 pose init | **LiLoc 没做，自写** | ~300 | SOLiD 候选 + small_gicp 粗配 |
| NDT-omp / GICP-omp 集成 | apt install `ros-noetic-ndt-omp` | ~100 | 胶水 |
| ROS service（save_map/save_session） | srv 定义抄 | ~150 | 薄壳 |
| **合计** | — | **~1600** | 3-4 周工程量 |

---

## 7. 意外发现

- **`priorNoise_ = diag(1e-12)`** — 几乎是硬约束。这意味着 LiLoc 对 prior session 的每个 keyframe 位姿**完全信任**。对我们温室跨季节场景（植被变化大）是否合适需要评估，可能要放松到 1e-4~1e-6。
- **`robustNoise_`（Huber kernel）** — 是处理 multi-session 跨时间匹配错误的关键。值得借。
- LiLoc 用 **ISAM2**（`relinearizeThreshold=0.1`）而非 batch LM，证明 **lifelong + 实时增量优化在工程上可行**。我们 C-1 的 ISAM2 基础可以直接扩展到 C-3。
- `BetweenFactorWithAnchoring` 的**数学源头**是韩国 giseop（Giseop Kim）2020 年在 LT-SLAM 里的实现，LiLoc 是 **搬用**，不是原创。这个 factor 工程成熟度比 LiLoc 本身高。如果担心 LiLoc 作者不维护，可以直接从 giseop 原始实现（LT-SLAM / SC-LIO-SAM）copy。

---

## 8. 决策建议

**C-3 路线**：
1. 不整包引入 LiLoc。
2. 借 3 个模块（`BetweenFactorWithAnchoring` / `AnchorOptimization` 思路 / `Session` 思路），配我们自写的 SOLiD + small_gicp 做 relocalizer。
3. 预期代码量 ~1600 行 / 3-4 周。
4. 保留 LiLoc 仓库在 `~/refs/` 长期参考。

**不做的**：
- LIO-SAM 风格特征提取（imageProjection）—— 我们前端用 FAST-LIO2 IESKF。
- LIO-SAM 风格 IMU 预积分（imuPreintegration）—— 已在 FAST-LIO2 里。
- ikd-Tree 副本 —— 我们已有。
- ParamServer —— 我们有 yaml schema。
- better_fastlio2 的 pose init —— GPL-2.0 不兼容，自写。

---

## 9. 下一步选项

- **C** — small_gicp vs nano_gicp 对比选一个
- **D** — 直接进入 C-1 实现：docker 加 GTSAM 层 + 写 `include/slam/graph/*.hpp`
- **E** — 再读 FAST_LIO_SLAM/SC-PGO 看 sidecar 边界
- **F** — 读 SOLiD 的 CPU C++ 实现（仅 269 行），评估 Mid360 descriptor 接入成本

回字母或组合。
