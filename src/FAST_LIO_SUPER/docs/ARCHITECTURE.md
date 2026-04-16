# FAST-LIO Super — Architecture

本文档描述 `fast_lio_super` 包在 FAST-LIO2 之上引入的分层结构。目的是让 Super-LIO 的
OctVox / HKNN / Relocalization 能以**可替换模块**的方式落进 FAST-LIO2，而不破坏
IESKF 的数学核心。

相关文档：
- 迭代路线：`docs/ITERATION_PLAN.md`
- 接口合同：`docs/SLAM_CORE_API.md`

---

## 1. 背景

- FAST-LIO2：IESKF + ikd-tree，单体 ROS 节点 `fastlio_mapping`。
- Super-LIO：在 FAST-LIO 主干上换地图表示（OctVox）和邻域搜索（HKNN），并
  用 LRU / Robin Hood hashing / parallel 实现降时延与控内存。
- 我们的目标不是"把 FAST-LIO2 换成 Super-LIO"，而是在 FAST-LIO2 的骨架上吸收
  Super-LIO 的 **MapManager + NeighborSearch + Relocalizer** 三件套。

## 2. 分层

```text
           ┌────────────────────────────────────────┐
ROS 层 →   │  laserMapping.cpp（保持 FAST-LIO2 形态）│
           └──────┬─────────────────────────────────┘
                  │
                  ▼  仅通过接口调用
           ┌────────────────────────────────────────┐
slam_core  │  IMapManager   INeighborSearch         │
(算法核)   │  ┌──────────┐  ┌──────────┐            │
           │  │ IkdTree  │  │ KdTreeKnn│  Phase A   │
           │  │   Map    │  │  (内嵌)  │            │
           │  └──────────┘  └──────────┘            │
           │  ┌──────────┐  ┌──────────┐            │
           │  │ OctVox   │  │  HKNN    │  Phase B   │
           │  │   Map    │  │ Search   │            │
           │  └──────────┘  └──────────┘            │
           │  ┌──────────┐                          │
           │  │Relocalizer│                 Phase B │
           │  └──────────┘                          │
           │  ┌──────────┐                          │
           │  │ Keyframe │                 Phase C │
           │  │  Mgr     │                          │
           │  └──────────┘                          │
           └────────────────────────────────────────┘
```

### 2.1 目录布局

```
src/FAST_LIO_SUPER/
├── include/
│   ├── slam/
│   │   ├── common_types.hpp          # PointT / Pose6D / Neighbor / PlaneCoef
│   │   ├── nn/neighbor_search.hpp    # INeighborSearch
│   │   ├── map/map_manager.hpp       # IMapManager + MapConfig/Stats
│   │   ├── map/ikd_tree_map.hpp      # Phase A 后端
│   │   ├── io/ …                     # Phase B：MapIO
│   │   ├── reloc/ …                  # Phase B：IRelocalizer
│   │   └── graph/ …                  # Phase C：IKeyframeManager
│   ├── common_lib.h / so3_math.h / use-ikfom.hpp / IKFoM_toolkit/  # FAST-LIO2 原件
│   └── ikd-Tree/                     # 原 ikd-tree，slam_core 内部复用
├── src/
│   ├── laserMapping.cpp              # FAST-LIO2 主节点，暂未改
│   ├── preprocess.{h,cpp}            # 原件
│   ├── IMU_Processing.hpp            # 原件
│   └── slam/
│       ├── map/ikd_tree_map.cpp      # Phase A 实现
│       └── …                         # 后续阶段
├── docs/
│   ├── ARCHITECTURE.md               # 本文件
│   ├── ITERATION_PLAN.md
│   └── SLAM_CORE_API.md
├── config / launch / msg / rviz_cfg  # 原件（pkg 名已改为 fast_lio_super）
└── CMakeLists.txt                    # 增 target: fast_lio_super_slam
```

### 2.2 CMake target

- `fastlio_super_mapping`：原 FAST-LIO2 可执行，**尚未**链接 `slam_core`。
- `fast_lio_super_slam`：静态库，包含 `IkdTreeMap` + `ikd_Tree.cpp`。Phase A-2
  时让 `fastlio_super_mapping` 改链到它。

### 2.3 ROS 层契约

- 节点名：`laserMapping`（与原 FAST-LIO 一致，便于 `greenhouse_nav` 不动）
- pkg 名：`fast_lio_super`；launch `roslaunch fast_lio_super mapping_mid360.launch`
- topic、tf、参数名：**不改**。

## 3. 组合矩阵（A/B 实验目标）

| 配置 | `active_map` | `persistent_map` | `search` |
|---|---|---|---|
| `fl2_baseline` | ikdtree | — | kdtree_knn |
| `fl2_star` | ikdtree | — | kdtree_knn + TBB |
| `fl2_octvox_sidecar` | ikdtree | octvox（旁路压缩/落盘） | kdtree_knn |
| `superlio_like` | octvox | octvox | hknn |

这四组解耦"并发收益 vs 地图管理收益"，复现 Super-LIO 论文里 FAST-LIO2* 的公平
比较逻辑。

## 4. 不做的事

- 不改 IESKF 数学（`esekfom.hpp` / `use-ikfom.hpp`）。
- 不改 topic/tf/参数名，`greenhouse_nav` 与下游 terrain/local planner 零感知。
- 不在 Phase A 引入 TBB；并发模型单独作为 Phase B 的独立 task。
- 不做全局 BA；Phase C 先轻 keyframe pose graph。
