# MoveBase3D — 3D 导航规划套件

**归属项目**: 项目 B — 温室自主导航系统（备选/参考导航管线）
**标签**: `C++` `Python` `RViz 插件` `3D Path Planning` `PCL`

---

## 概览

包含从全局规划到局部执行、从传感器处理到 RViz 可视化插件的完整 3D 导航套件。与 FAR Planner 生态（far_planner, local_planner, terrain_analysis）为平行/备选方案。

### 子包结构

```
MoveBase3D/
├── global_planner_3d/       # 3D 全局规划器（C++）
├── local_planner_3d/        # 3D 局部规划器（Python）
├── rover_gpr/               # 高斯过程回归地形分析
├── rover_launch/            # 一键启动入口
├── rover_msgs/              # 自定义消息（BMS/目标/遥控状态）
├── rover_pcl_filter/        # 点云滤波器
├── rover_teleop/            # 键盘遥控
└── rviz_plugins/            # RViz 可视化插件
    ├── rviz_goal_tool_plugins/           # 目标点/位姿工具
    ├── rviz_multi_goals_manager_plugins/ # 多点管理器
    └── rviz_waypoint_generator_plugins/  # 路径点生成
```

---

## 子包详情

### global_planner_3d

C++14 全局 3D 路径规划器。

```
include:
  execution_classes.h   — 规划执行类定义
  execution_planner.h   — 规划器接口
  execution_vis.h       — 可视化
  backward.hpp          — Debug 回溯

src:
  global_planning_node.cpp  — 主节点（订阅 waypoints + cloud_map）
  global_planning_obs.cpp   — 障碍物检测节点
  execution_planner.cpp     — 路径搜索核心
  execution_classes.cpp     — 数据结构实现
  execution_vis.cpp         — RViz 可视化
```

| 订阅 | `/waypoint_generator/waypoints`, `/rtabmap/cloud_map` |
|------|-------------------------------------------------------|
| 发布 | 全局路径 (`nav_msgs/Path`) |

### local_planner_3d

Python 局部路径规划器（基于 DWA 的 3D 版本）。

```
scripts/local_planner_node.py — 局部规划主节点
```

### rover_gpr

高斯过程回归地形分析。

```
include/gaussian_process_regression/
  gaussian_process_regression.h  — GPR 核心

src/gpr_path.cpp  — GPR 路径规划节点
```

基于 GP 回归对地形高程进行插值与不确定性估计，生成可通行性概率地图。

### rover_pcl_filter

点云滤波与管理。

```
src/pcl_filter_manager.cpp — 点云滤波器节点
```

| 订阅 | 原始 PointCloud2 |
|------|-------------------|
| 发布 | 滤波后 `/output_filter` |

融合多种滤波器（体素降采样、离群点移除、直通滤波）。

### rover_msgs

自定义消息类型：

| 消息 | 用途 |
|------|------|
| `roverBmsStatus.msg` | 电池管理系统状态 |
| `roverGoalStatus.msg` | 导航目标状态 |
| `roverRCStatus.msg` | 遥控器状态 |

### rviz_plugins

三种 RViz 可视化插件：

| 插件 | 功能 |
|------|------|
| `rviz_goal_tool_plugins` | 3D 目标点/位姿投放、概率地图显示、航拍地图叠加 |
| `rviz_multi_goals_manager_plugins` | 多点导航任务管理面板 |
| `rviz_waypoint_generator_plugins` | 路径点序列生成/编辑 |

### rover_teleop

键盘遥控节点（`setup.py` + `rover_teleop/__init__.py`）。

---

## 与主导航管线的关系

| 维度 | 主管线 (FAR Planner) | 备选 (MoveBase3D) |
|------|---------------------|-------------------|
| 全局规划 | far_planner (visibility graph A*) | global_planner_3d |
| 局部规划 | local_planner (CMU Trajectory Rollout) | local_planner_3d (DWA) |
| 地形分析 | terrain_analysis (elevation mapping) | rover_gpr (GP regression) |
| 状态 | 生产中 | 备选/参考 |
