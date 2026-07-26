# Phase C 后端选型报告

**日期**：2026-04-18
**作者**：claude (基于实际 clone + 源码审阅)
**目的**：为 `fast_lio_super` 挑选 Phase C 的后端蓝本。先回答"Super-LIO 有没有后端"，再对比 3 个主流候选，最后给推荐。

---

## 0. 问题起点

用户问："我们应该选 Super-LIO 的后端？还是其他？"

**答：Super-LIO 没有后端。** 它和 FAST-LIO2 是同一类产品（前端 LIO + 地图管理 + relocalization）。
```bash
$ find /tmp/refs/Super-LIO-ros1 -name "*.hpp" -o -name "*.cpp" | \
    xargs grep -l "pose_graph\|gtsam\|factor_graph\|loop_closure\|keyframe"
# 无输出
```
证据：`src/super_lio/include/lio/` 里只有 `ESKF / params / super_lio / super_lio_reloc` 四个头文件，没有任何 graph / keyframe / loop 相关符号。Super-LIO 论文 `arXiv:2509.05723` 的贡献声明也是 "compact mapping strategy"，不涉及全局优化。

所以 Phase C 的后端**必须从别的工程借**。

---

## 1. 候选池（已真实 clone 并审过源码）

### 1.1 LIO-SAM — `TixiaoShan/LIO-SAM`

| 项 | 值 |
|---|---|
| License | BSD-3 |
| 代码量（后端相关） | `mapOptmization.cpp` 1779 行 + `imuPreintegration.cpp` 524 行 = **2303 行** |
| 优化器 | GTSAM ISAM2 |
| 因子类型 | `PriorFactor` / `BetweenFactor` / `GPSFactor` / `CombinedImuFactor` |
| 关键帧触发 | `saveFrame()`：距离 > thres 或姿态变化 > thres |
| 回环检测 | ICP-based（`performLoopClosure`），独立后台线程 `loopClosureThread` |
| 系统地位 | ROS1 经典方案，被大量项目 fork |
| 依赖 | GTSAM（**不在 Ubuntu 20.04 apt，需源码编译**）+ PCL + Eigen |
| 前端 | 自带 LOAM 风格特征提取（`featureExtraction.cpp` + `imageProjection.cpp`），**我们不用** |

**接入点**：
```cpp
// LIO-SAM mapOptmization.cpp 关键函数
bool saveFrame();                 // 关键帧触发判定
void addOdomFactor();             // 相邻关键帧 BetweenFactor
void addImuFactor();              // IMU 预积分因子（来自 imuPreintegration 线程）
void addGPSFactor();              // 可选 GPS
void addLoopFactor();             // 回环因子（ICP 对齐后的相对位姿）
void performLoopClosure();        // ICP 在历史关键帧周围找回环
void loopClosureThread();         // 独立线程
// 入口
void saveKeyFramesAndFactor() {
  if (saveFrame() == false) return;
  addOdomFactor();
  addImuFactor();
  addGPSFactor();
  addLoopFactor();
  // ISAM2 update
}
```

**移植策略**：只抄 `mapOptmization.cpp` 的后端部分（~800 行）。**不抄** `imageProjection.cpp`（LOAM 去畸变，我们用 FAST-LIO2 的 IMU 积分）和 `featureExtraction.cpp`（特征提取，我们用原始点云）。`imuPreintegration.cpp` 可以**直接抄**，GTSAM PreintegratedImuMeasurements 是标准库，与我们 FAST-LIO2 的 IMU 流独立并行。

---

### 1.2 Voxel-SLAM — `hku-mars/Voxel-SLAM`

| 项 | 值 |
|---|---|
| License | **GPLv2** |
| 代码量（后端） | `voxelslam.cpp` 2625 行 + `BTC.cpp` 1479 行 + headers = **> 4000 行** |
| 优化器 | 自研 hierarchical voxel BA（**GTSAM 代码被注释掉了**，看 `voxelslam.cpp:410-440`） |
| 因子类型 | Plane-to-plane voxel BA + STD (Stable Triangle Descriptor) 回环 |
| 关键帧触发 | 距离 + 视角变化 |
| 回环检测 | 自研 BTC (Binary Triangle Context) 描述子 + 几何验证 |
| 系统地位 | SOTA 精度，但架构重、代码可读性一般 |
| 依赖 | PCL + Eigen + Ceres (BA solver) |
| 前端 | 自己的 voxel map 贯穿前后端，**与我们 OctVox 方向兼容但不是同一个** |

**问题**：
- Hierarchical BA 要求前后端共享同一 adaptive voxel map；强行插进来等于重写前端地图层。
- Voxel-SLAM 的 voxel 是"为 BA 设计"的（每个 voxel 拟合平面/线），Super-LIO OctVox 是"为快速 KNN 设计"的（sub-voxel 均值代表点）。两者不兼容，除非再做一层转换。
- License GPL**v2**（不是 v3），技术上与我们现在的 GPLv3 存在"后向不兼容"争议——GPLv2-only 代码无法并入 GPLv3 项目。需要看 LICENSE 首行是 "version 2" 还是 "version 2 or later"。
  - 实测：`/tmp/refs/Voxel-SLAM/LICENSE` 第二行 `Version 2, June 1991`，没有 "or later" → **纯 GPLv2**，与我们 GPLv3 组合需要法律咨询。

**判断**：Phase C **不推荐**。作为 Phase D+ 的长期升级路径可以再看。

---

### 1.3 hdl_graph_slam — `koide3/hdl_graph_slam`

| 项 | 值 |
|---|---|
| License | BSD-2 |
| 代码量（后端） | `hdl_graph_slam_nodelet.cpp` 1111 行 + `graph_slam.hpp/cpp` ~500 行 + `loop_detector.hpp` 188 行 = **~1800 行** |
| 优化器 | g2o (`g2o::SparseOptimizer`) |
| 因子类型 | SE3 Between Edge + Ground Plane Edge + GPS Edge + Loop Edge |
| 关键帧触发 | `keyframe_updater.hpp`：距离阈值 |
| 回环检测 | GICP/NDT scan-to-scan（基于 `fast_gicp` + `ndt_omp`） |
| 系统地位 | 日本 AIST 维护，活跃度稳定，注重结构化环境 |
| 依赖 | g2o (apt 有 `ros-noetic-libg2o`) + fast_gicp + ndt_omp（均有 apt 包或 catkin 源） |
| 前端 | **自带 scan_matching_odometry_nodelet，但可完全不用**（接受外部 odometry topic） |

**系统架构**（`hdl_graph_slam_nodelet` 是 ROS nodelet）：
```
输入 topics:
  /odom        ← 我们可以用 FAST-LIO2 的 /Odometry 替代
  /filtered_points ← FAST-LIO2 的 /cloud_registered 可复用
  /gps/fix     ← 可选
  /imu/data    ← 可选

输出:
  /hdl_graph_slam/map_points   ← 全局优化后的点云地图
  /hdl_graph_slam/odom2pub     ← 优化后的位姿
  /hdl_graph_slam/graph_edges  ← 可视化
```

**关键优势**：**设计上就是"独立后端"**，不侵入前端。最接近"直接插在 FAST-LIO2 后面"的拓扑。

**风险**：
- 主打 Velodyne/HDL，温室 Mid360 的场景特性（窄 FoV、稀疏点）要调参
- 回环依赖 GICP，对 FOV 差异敏感
- 代码是 nodelet 架构，调试略比 node 麻烦

---

### 1.4 作为参考但不推荐的候选

**SC-LIO-SAM**：基于 LIO-SAM + ScanContext 回环。如果选 LIO-SAM 路线，后期可以把回环层单独换成 SC，作为 B-4 的优化。

**BALM 2.0**：纯 BA 工具，不带 keyframe 管理和 graph，需要外壳包装。适合 Phase D 的 BA 升级，不适合 C 的骨架。

**ct_icp**：连续时间轨迹 + ICP，走的不是 graph 路线，与我们 IESKF 前端语义不匹配。

---

## 2. 重要依赖核查

### 2.1 GTSAM 在 Ubuntu 20.04 Noetic 的可得性

```
$ docker run --rm fastlivo-dev:noetic-ros-base apt-cache search gtsam
(空)
```

**GTSAM 不在 focal 官方仓库，也不在 ROS1 Noetic**。LIO-SAM 的 README 原话：
> Follow GTSAM's INSTALL instructions to build and install GTSAM from source. We tested on GTSAM 4.0.2.

源码编译路径（30 分钟）：
```bash
git clone -b 4.0.3-alpha0 https://github.com/borglab/gtsam.git
cd gtsam && mkdir build && cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
make -j8 && sudo make install
```

**这意味着 docker 编译镜像要加一层 GTSAM。** 我们已有的 `fastlivo-dev:noetic-ros-base` 没装，新镜像 `fastlivo-memopt-dev` 也没装（见 B-1 时的 docker images）。

### 2.2 g2o 在 Ubuntu 20.04 Noetic

```
$ apt-cache search ^libg2o
ros-noetic-libg2o ...  # 存在
```

**g2o 有 apt 包**，`sudo apt install ros-noetic-libg2o` 即可。hdl_graph_slam 依赖即用型，不用源码编译。

---

## 3. 工程量对比

| 候选 | 不装新依赖的工作量 | 装新依赖后的工作量 | 移植 Adapter 代码量 | 回环功能从 |
|---|---|---|---|---|
| LIO-SAM | 不可行（需 GTSAM） | GTSAM 源码编 30 min + 抄 `mapOptmization.cpp` 部分 ~800 行 + 写 `imuPreintegration` 接入 | ~800-1000 行 | 自带 ICP loop closure |
| Voxel-SLAM | 不可行（GPL v2 license 风险） | — | > 3000 行（整个系统） | 自带 STD 回环 |
| hdl_graph_slam | **可行**，apt 装 g2o / fast_gicp / ndt_omp 后直接 `catkin_make` | — | **~200-400 行**（只做 odometry topic 桥接） | 自带 GICP loop closure |

---

## 4. 与现有架构的契合度

### 4.1 我们的 `slam_core` 接口是否要扩展

**LIO-SAM 方案**：需要新增 `slam_core/graph/IKeyframeManager + IPoseGraph` 接口，把 LIO-SAM 的 `saveFrame/addOdomFactor/addLoopFactor` 抽象出来。约 300 行接口头 + 800 行实现。

**hdl_graph_slam 方案**：基本**不需要改 `slam_core`**。hdl_graph_slam 作为独立 nodelet，消费 FAST-LIO2 的 `/Odometry` + `/cloud_registered`，产出全局优化后的 topic。接入方式是 launch 文件层的组合。

### 4.2 与 OctVoxMap 的耦合

**LIO-SAM 方案**：后端的"周围关键帧地图"是用 ikd-Tree 维护的，与 OctVox 独立。不打架。但若 future 要让后端读 OctVox 的代表点，要改 LIO-SAM 代码。

**hdl_graph_slam 方案**：后端有自己的 pose graph-only map，不依赖前端地图结构。零耦合。

### 4.3 与 greenhouse_nav 主线

**两者都输出标准 `nav_msgs/Odometry`，下游（terrain_analysis / far_planner / local_planner）都能消费**。但 hdl_graph_slam 的 topic naming 更友好，不用改 greenhouse_nav。

---

## 5. 决策矩阵

| 维度 | LIO-SAM | Voxel-SLAM | hdl_graph_slam |
|---|---|---|---|
| License 友好度 | ★★★★ | ★ | ★★★★★ |
| 依赖就绪（Noetic apt） | ★ | ★ | ★★★★★ |
| 前端解耦 | ★★★ | ★ | ★★★★★ |
| 精度上限 | ★★★★ | ★★★★★ | ★★★ |
| 回环检测质量 | ★★★★ | ★★★★★ | ★★★ |
| 后端可裁剪性 | ★★★★ | ★★ | ★★★★ |
| 代码可读性 | ★★★ | ★★ | ★★★★ |
| 社区/活跃度 | ★★★★★ | ★★★ | ★★★★ |
| 工程量（到可跑） | 大 | 巨大 | 小 |
| 与 FAST-LIO2 的拓扑契合 | ★★★ | ★ | ★★★★★ |

---

## 6. 推荐

**分两步走**：

### 阶段 C-1（MVP，1-2 周）：采用 hdl_graph_slam
- 理由：最快让系统"有一个后端"，不改前端；前端 FAST-LIO2 + OctVox 继续按我们已经做完的那一套跑
- 动作：
  1. `apt install ros-noetic-libg2o ros-noetic-ndt-omp ros-noetic-fast-gicp`
  2. 把 `hdl_graph_slam` 整包作为 submodule 加进 `src/`
  3. 写一个 bridge node（~100 行）把 `/Odometry`（FAST-LIO2）转成 hdl 需要的 `/odom`
  4. launch 层：`greenhouse_navigation.launch` 增加 `<include file="$(find hdl_graph_slam)/launch/hdl_graph_slam.launch" />`
- 交付：有关键帧 pose graph、有回环（GICP）、有全局优化后的位姿输出
- 代价：精度不如 LIO-SAM / Voxel-SLAM，回环覆盖率一般

### 阶段 C-2（成熟，3-4 周）：替换成 LIO-SAM 的 mapOptmization
- 理由：C-1 验证"后端+FAST-LIO2"的拓扑可行后，换 GTSAM + ISAM2，精度和回环质量都升级
- 前置：docker 镜像加 GTSAM 层（一次性工作）
- 动作：
  1. 把 `mapOptmization.cpp` 的后端部分抽到 `slam_core/graph/`（keyframe + pose graph + ISAM2）
  2. `imuPreintegration.cpp` 直接抄，与 FAST-LIO2 IMU 积分并行
  3. 保留 hdl_graph_slam 作为 fallback（yaml 可切）
- 交付：论文级精度，可作为长期后端

### 不推荐的阶段 C：直接上 Voxel-SLAM
- 工程量失控（> 3000 行改动）
- License GPLv2 与我们 GPLv3 存在不兼容风险
- 需要重做前端地图层

---

## 7. 风险清单（两条路线共有）

1. **Mid360 的稀疏性**：回环检测（无论 ICP / GICP）对稀疏点云都不友好。温室这种相对结构化环境可能还好，但林地/坡地的 Mid360 数据需要回环参数专项调优。
2. **关键帧触发阈值**：默认值（LIO-SAM 1m/10°，hdl 2m/10°）对温室这种小尺度场景太粗，需要降到 0.3-0.5m。
3. **GTSAM 版本锁定**：LIO-SAM 要求 4.0.x，不要装 4.2+（API 不兼容）。docker 里 FROM GTSAM 源码编译要固定 tag。
4. **g2o 版本**：Ubuntu 20.04 的 `ros-noetic-libg2o` 已足够新（2020 年），hdl_graph_slam 验证过能跑。
5. **回环导致 "map jump"**：后端优化一旦插入回环因子，历史位姿会跳变。如果前端 FAST-LIO2 继续用旧坐标系，`tf` 会断裂。需要在 bridge 层做 "world2map_correction" TF 补偿——这是所有 LIO + 后端方案的通病。

---

## 8. 需要你拍板的点

1. **C-1 走 hdl_graph_slam 先出 MVP？**（推荐）
2. 如果直接想一步到位上 LIO-SAM，接受 GTSAM 源码编译 + docker 镜像升级？
3. Voxel-SLAM 的 GPLv2 风险——是否能接受"GPL v2 vs v3 合规性再查"再决定？
4. 两条路线都要吃的坑：关键帧阈值调小到温室尺度（0.3-0.5m），是否有共识？

回我 `1-4` 的决策，我接着动手。

---

## 附录 A：代码量口径

- LIO-SAM `mapOptmization.cpp` 1779 行，但其中 ~40% 是 ROS topic 发布/可视化/RViz 相关代码，实际算法 ~1000 行；我们需要的"keyframe + factor + ISAM2"部分约 600-800 行。
- hdl_graph_slam 作为 submodule 零抄写，只写 bridge ~100 行。
- Voxel-SLAM 难以裁剪，前后端耦合深，全盘引入才有效。
