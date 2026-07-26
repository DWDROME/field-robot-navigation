# Phase C 后端选型报告 v2

**日期**：2026-04-18
**作者**：Claude（基于 ChatGPT 深度调研报告 + 两轮内部评估合成）
**取代**：`BACKEND_SELECTION_REPORT.md`（v1 视野过窄，已作废）

本文是 v1 选型报告的**替代版**。v1 把问题压成 "LIO-SAM vs hdl_graph_slam vs Voxel-SLAM" 三选一，ChatGPT 的 30 题深度调研给出了系统性修正。把结论写在前面，再给对比与理由。

---

## 0. 核心结论（一句话）

**不要再做三选一。走"拼装式系统工程"路线：借 FAST-LIO 社区 sidecar 的 keyframe/PGO 主通路 + 借 MOLA 的 simple-map 分层 + 借 LiLoc 的长期定位 + Mid360 场景下 ScanContext 必须被更合适的 descriptor 替换。**

---

## 1. v1 方案 vs 深度调研方案对比

| 维度 | v1 报告结论 | 深度调研修正 |
|---|---|---|
| 候选池 | LIO-SAM / Voxel-SLAM / hdl_graph_slam 三选一 | 扩到 20+，核心是 FAST-LIO sidecar 族群 |
| C-1 MVP 推荐 | hdl_graph_slam（apt 依赖零成本） | **不推荐**。生态脱节。改走 FAST_LIO_SLAM / FAST-LIO-SAM 图纸 + 自写后端 |
| 回环 descriptor | 默认 ScanContext | **必须插件化**。Mid360 + 林地下 ScanContext 判别性不足，CPU 基线应是 **SOLiD 或 STD** |
| 重定位 | FAST_LIO_LOCALIZATION 或自写 | **LiLoc**（MIT，lifelong localization，adaptive submap joining）；FAST_LIO_LOCALIZATION 依赖 Python 2.7 + Open3D 0.9，是旧栈 |
| 规划地图 sidecar | 未讨论 | CPU-only 方案：**Bonxai (MPL-2.0) / Rolling-Bonxai**；G-VOM 是 GPU 不适合 |
| ROS2 迁移 | 未结论 | 核心设计做 ROS-agnostic，外围 ROS1 封装；**迁移不能和后端同时做**（三个月窗口耦合风险过高） |
| 首版后端范围 | 倾向一步到位 | 最小切面：**keyframe + 相邻因子 + iSAM2 + map correction bridge**，第一版**不做 BA**（HBA/BALM 是后处理） |

---

## 2. 许可证最终盘点（深度报告已核实部分）

**可用（BSD / MIT / Apache-2.0 / MPL-2.0 / GPLv3）**：
- GLIM (MIT)、LiLoc (MIT)、Hybrid-VoxelMap (MIT)
- LTA-OM (Apache-2.0)
- SuperOdom (GPLv3)
- Bonxai (MPL-2.0)
- R-VoxelMap (BSD-3)
- KISS-SLAM (MIT)

**只能读代码，不能并库**：
- FAST-LIO-SAM、FAST-LIO-SAM-QN（**CC-BY-NC-SA 4.0**，NonCommercial）
- better_fastlio2（**GPL-2.0**，与我们 GPLv3 不兼容，FSF 官方确认）
- FAST_LIO_LOCALIZATION（GPL-2.0，同上）
- Voxel-SLAM（GPL-2.0，同上）
- VoxelMap（GPL-2.0，且 README 注明商业用途需联系作者）

**未知 / 需手工核 LICENSE 文件**：
- FAST_LIO_SLAM、FAST_LIO_LC、FASTLIO2_ROS2
- MOLA、VoxelMapPlus_Public、C3P-VoxelMap
- FAST-LIO-SAM-SC-QN

**结论**：**许可证不明的一律先当"读代码学结构"的参考**，不进入 main branch 依赖图。

**重要澄清**：better_fastlio2 的 GPL-2.0 意味着即使我们项目已经是 GPLv3，**也不能并库**（GPLv2-only 不向前兼容 GPLv3）。这是 v1 报告没识别的风险。

---

## 3. Mid360 + 林地场景的回环方案

深度报告的关键数据：

**硬事实**：
- 2024 dense forest LiDAR PR 评测：Logg3dNet 最佳，5m 基线 80% / 10m 基线 60% 正确 loop
- 2025 ForestLPR：Recall@1 比最近 baseline 平均 +7.38% / +9.11%
- BEVPlace++：RTX 3090 + i5-7500 上 40Hz 可达，模型 17MB——但**无 CPU-only Mid360 实测证据**
- SOLiD：专门为 FOV-constrained LiDAR 设计，明确把 Livox 作典型场景

**明确的未知**：
- Mid360 单帧 scan-to-scan GICP 回环成功率——**没有论文级统计**

**工程推断**：
- Mid360 窄 FoV + 非重复扫描下，loop 应该是 **descriptor-first + geometry-second** 两阶段，不是"单帧 + 固定 GICP"
- accumulated submap 回环比单帧更稳

**候选优先级**：
- CPU 基线：**SOLiD**（首选，FOV-aware）或 **STD**（handcrafted 成熟）
- 林地插件：**ForestLPR** 或 **TreeLoc++**
- 学习式可选：**BEVPlace++**（若有 GPU 或能接受 CPU 推理时延）
- ScanContext 退为 baseline / fallback，**不再是默认**

---

## 4. 推荐的 Phase C 架构（深度报告版）

```
前端（保留不改）
  FAST-LIO2 IESKF + ikd-tree | OctVox（yaml 切换）
        │
        ├─▶ Active Registration Map（已完成）
        │
        ▼
关键帧触发
  IKeyframeManager（新增，P0）
        │
        ├─▶ View-based Keyframe Store（新增，P0）
        │      ├─ raw scan 或下采样后的点云
        │      ├─ 位姿
        │      ├─ descriptor 索引
        │      └─ 时间戳 / session id
        │
        └─▶ ILoopCandidateProvider（插件化，P0）
               ├─ SOLiD / STD（CPU 基线）
               ├─ ForestLPR / TreeLoc（林地插件）
               └─ BEVPlace++（可选 GPU）
                     │
                     ▼
              ILoopVerifier（P0）
                 Nano-GICP / small_gicp（精配准）
                 Quatro（可选，大错位粗初值）
                     │
                     ▼
              IPoseGraph（GTSAM iSAM2，P0）
                     │
                     ▼
              MapCorrectionBridge（P0）
                 ├─ 回写前端 T_map_odom 补偿（低侵入，推荐）
                 └─ 或直接更新前端 state（侵入，FAST_LIO_LC 路线）
                     │
                     ▼
从 Keyframe Store 异步派生
  Planning Voxel Sidecar（P1）
     ├─ Bonxai / Rolling-Bonxai（occupancy）
     ├─ Height map / traversability（未来）
     └─ ESDF（可选）

Relocalizer（P1）
  LiLoc（主线）—— 不走 FAST_LIO_LOCALIZATION 老栈
```

---

## 5. Phase C 分期

### C-1（3-4 周）：最小可行后端骨架

**目标**：让系统拥有 "FAST-LIO2 + keyframe + PGO + 单次回环 + map correction bridge"。不求最优回环质量，只求接口通路全走一遍。

**动作**：
1. docker 镜像加 GTSAM 4.0.3 源码编译层（~30min 一次性成本）
2. 写 `slam_core/graph/keyframe_manager.{hpp,cpp}`（~300 行）
3. 写 `slam_core/graph/pose_graph.{hpp,cpp}`（iSAM2 wrapper，~400 行）
4. 写 `slam_core/graph/loop_candidate_provider.hpp`（接口 + ScanContext 最小实现，~200 行）
5. 写 `slam_core/graph/loop_verifier.hpp`（Nano-GICP wrapper，~150 行）
6. 写 `slam_core/graph/map_correction_bridge.{hpp,cpp}`（~200 行，走 TF 补偿路线）
7. `laserMapping.cpp` 注入 keyframe 触发钩子（~50 行 diff）
8. yaml 扩展 `slam_core/backend:` 子段

**代码量预估**：~1500 行 + GTSAM 依赖

**参考图纸**：FAST-LIO-SAM（engcang）— 只读不并（CC-BY-NC-SA）

**退出条件**：回放 bag → 看到 keyframe 发布、ISAM2 增量优化、至少一次回环 factor 成功插入、优化后 `/Odometry_corrected` 正常发布、前端 tf 不跳变。

### C-2（4-6 周）：Mid360/林地适配与插件化

**目标**：默认 descriptor 从 ScanContext 升级到 SOLiD/STD，回环质量可实际用于林地场景。

**动作**：
1. SOLiD or STD 实现集成（作为 descriptor 插件）
2. accumulated submap 回环（不再是单帧）
3. ForestLPR 或 TreeLoc++ 作为可选插件（观察状态）
4. 关键帧阈值做成 `scene_profile`（温室 / 林地 / 丘陵）

**退出条件**：温室 bag 与林地 bag 分别跑通，回环误检率对比基线降低。

### C-3（2-3 周）：长期地图 + 重定位

**动作**：
1. LiLoc 或等价 submap-joining 方案接入
2. MapIO ROS 服务：save/load keyframe store
3. Localization-only 模式

### D（以后）：规划 sidecar + 一致性地图

- Bonxai / Rolling-Bonxai occupancy/traversability
- HBA / GLIM 级别的离线一致性地图优化

---

## 6. 仍有的三个不确定点

1. **FAST_LIO_SLAM 和 FAST_LIO_LC 的 LICENSE 需要手工看仓库根目录**。深度报告说"没拿到清晰信号"。这决定了我们能不能直接借它们的 sidecar 代码而不是自写。

2. **SOLiD 的 CPU-only 实测证据**：深度报告给出了论文与仓库线索，但没给"在 Mid360 bag 上跑出来的数字"。需要实测。

3. **map correction bridge 走 TF 补偿 vs 回写前端 state**：FAST_LIO_LC 走的是回写路线，但有 Z 漂移和 double free 问题。保守选择是 TF 补偿。深度报告倾向 TF。

---

## 7. 比 v1 报告更明确的事

- **hdl_graph_slam 从主线候选撤出**。它只能作为"纯独立后端参考架构"，不是 Phase C 落地方案。
- **Voxel-SLAM 彻底移出候选**。GPL-2.0 确认，与我们 GPLv3 不兼容。
- **better_fastlio2 不能并库**（GPL-2.0）。但可以读它的设计。
- **FAST-LIO-SAM 系列只能作图纸**（CC-BY-NC-SA）。要自己重写接口。
- **LiLoc 是重定位主线**，不是 FAST_LIO_LOCALIZATION。
- **Bonxai 是规划侧车的 CPU 方案**（MPL-2.0，相对友好）。
- **GTSAM 源码编译的 docker 层是必需的**。没有绕过路径。

---

## 8. 需要你拍板的点

**核心决策**：
1. **接受 Phase C 架构（第 4 节图）**：三图分离 + keyframe store + 插件化回环 + LiLoc 重定位 + Bonxai sidecar？
2. **C-1 从 3-4 周最小骨架开始**（不做 BA、回环用 ScanContext 起步、只打通通路）？还是直接一步到位 C-1+C-2？
3. **map correction 走 TF 补偿路线**（推荐，低侵入）？还是回写前端 state（FAST_LIO_LC 路线，更彻底但有风险）？

**二级决策**：
4. 第一版 descriptor：ScanContext（图纸成熟但不适配 Mid360）先跑通，C-2 再换 SOLiD / STD？还是直接上 SOLiD？
5. docker 镜像加 GTSAM 层，你来加还是我加？
6. ROS2 迁移暂缓到 Phase D 之后？核心模块做成 ROS-agnostic？

回复格式建议：`1=Y, 2=最小骨架, 3=TF, 4=先 SC 后 SOLiD, 5=我, 6=Y`。如果想要部分展开再说。

---

## 9. 参考文件

- `docs/CHATGPT_DEEP_RESEARCH_BRIEFING.md` — 给 ChatGPT 的原始 briefing
- `docs/DEEP_RESEARCH_REPORT_V2.md` — ChatGPT 返回的完整深度报告（30 题答复）
- `docs/BACKEND_SELECTION_REPORT.md` — v1 狭窄选型报告（作废，保留作对比）
- `docs/ITERATION_PLAN.md` — 待按本报告更新 Phase C 章节
