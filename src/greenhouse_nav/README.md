# greenhouse_nav

`greenhouse_nav` 是当前温室导航主线的**集成包**。  
它不负责重新实现 SLAM、FAR 或 CMU local planner，而是把这些已有能力装配成一条可启动、可回放、可部署的统一链路。

## 1. 职责

本包当前承担四类职责：

1. **系统入口收口**
   - 提供建图入口：`greenhouse_mapping.launch`
   - 提供导航入口：`greenhouse_navigation.launch`
   - 提供显式导航模式入口：`greenhouse_navigation_forward.launch` / `greenhouse_navigation_reverse.launch`
   - 保留 `system_greenhouse.launch` 作为兼容别名
2. **定位状态统一输出**
   - 通过 `localization_manager.py` 对外发布 `/state_estimation`
   - 输出语义是 `map -> base_link`，不是原始传感器 `body`
3. **控制消息桥接**
   - 通过 `cmd_vel_stamped_bridge.py` 将 `/cmd_vel_stamped` 转成 `/cmd_vel`
4. **兼容性保留**
   - 保留 `global_to_local_adapter.py`，但默认主线不启用

## 1.1 Mid360 安装姿态约定

- 当前实机 Mid360 是**向前下倾 27°**
- 这在 ROS 坐标里属于**绕 Y 轴的 pitch**
- 当前采用 **方案 A**：
  - 保留 FAST-LIO 原生 `camera_init -> body`
  - 新增 `body -> body_foot`
  - 新增 `camera_init -> robot_foot_init`
  - 现有导航兼容仍保留 `body -> base_link`
- 当前前下倾 27° 的补偿统一按：
  - **`pitch = -27° = -0.471239 rad`**

这里要特别区分：

- `body`：FAST-LIO 输出的传感器 / IMU body 语义
- `body_foot`：机器人水平辅助机体帧
- `robot_foot_init`：机器人水平辅助世界帧
- `base_link`：为现有导航下游保留的兼容机体帧

当前主线已把这层固定安装/辅助帧逻辑收敛到：

- `launch/system_greenhouse_base.launch`
- `scripts/localization_manager.py`

而不是：

- Livox 驱动外参
- FAST-LIO 的 LiDAR↔IMU 内参

验证时要注意：

- `camera_init` 基本继承传感器启动时姿态
- 所以即使安装外参改对了，只要 RViz 的 `Fixed Frame` 仍然是 `camera_init`，`/cloud_registered` 依旧可能看起来前下倾
- 当前建图 RViz 默认切到 **`Fixed Frame = robot_foot_init`**
- 这不是改 FAST-LIO 世界帧，而是给上层增加一层更符合机器人语义的水平参考系

## 2. 当前推荐入口

### 建图入口

```bash
roslaunch greenhouse_nav greenhouse_mapping.launch
```

对应文件：

- `launch/greenhouse_mapping.launch`

该入口默认固化：

- Mid360 驱动
- FAST-LIO 建图
- greenhouse 专用 RViz
- 固定安装外参 TF

其中 RViz 默认使用：

- `rviz/greenhouse_mapping_base_link.rviz`
- `Fixed Frame = robot_foot_init`

这样建图时先看到的是机器人水平辅助世界帧，而不是 FAST-LIO 默认的 `camera_init` 初始姿态语义。

### 导航入口

```bash
roslaunch greenhouse_nav greenhouse_navigation.launch
```

对应文件：

- `launch/greenhouse_navigation.launch`
- `launch/greenhouse_navigation_forward.launch`
- `launch/greenhouse_navigation_reverse.launch`

该入口默认固化：

- `use_route_planner := true`
- `use_livox := true`
- `use_localization := true`
- `use_fastlio_mapping := true`
- `use_global_localization := true`
- `use_map_publisher := true`
- `use_adapter := false`
- `use_base_autonomy := true`
- `use_terrain_analysis := true`
- `use_terrain_analysis_ext := true`
- `use_graph_decoder := false`
- `use_far_rviz := true`
- `use_base_serial := true`
- `localization_rviz := false`

也就是说，日常操作层现在只需要记两个命令：建图一个，导航一个。
如果当前实机传感器 / `body` 与车头正好反向，则直接改用：

```bash
roslaunch greenhouse_nav greenhouse_navigation_reverse.launch
```

它只会给导航链使用的 `body -> base_link` 追加 `180° yaw`；
`greenhouse_mapping.launch` 建图链不受影响。

### 兼容入口

```bash
roslaunch greenhouse_nav system_greenhouse.launch
```

这个旧命令仍可用，但现在只是导航入口别名。

## 3. 建图与导航的关系

### 3.1 当前默认主线不是“开机即自由导航”

当前默认主线是：

```text
先建图 -> 再导航
```

也就是说，`greenhouse_navigation.launch` 默认依赖一张预建好的全局点云地图。

### 3.2 默认依赖的地图产物

当前默认地图路径是：

```bash
$(find fast_lio)/PCD/scans.pcd
```

在当前工作空间里，对应通常就是：

```bash
src/FAST_LIO/PCD/scans.pcd
```

### 3.3 导航前还需要什么

- `/map`：由 `pcd_to_pointcloud` 发布预建地图
- `/initialpose`：给全局重定位一个粗初值
- `/cloud_registered`
- `/Odometry`

如果没有预建地图或没有初始位姿，默认导航链路不会完整进入可用状态。

## 4. 启动链路

```text
greenhouse_navigation.launch
  -> system_greenhouse_with_route_planner.launch
     -> system_greenhouse_base.launch
        -> livox_ros_driver2
        -> FAST_LIO
        -> FAST_LIO_LOCALIZATION
        -> localization_manager.py
        -> terrain_analysis
        -> localPlanner
        -> pathFollower
        -> cmd_vel_stamped_bridge.py
        -> dlrobot_robot/base_serial.launch
     -> terrain_analysis_ext
     -> far_planner

greenhouse_mapping.launch
  -> livox_ros_driver2
  -> FAST_LIO/mapping_mid360.launch
  -> FAST-LIO RViz
  -> fixed TF publishers

system_greenhouse.launch
  -> greenhouse_navigation.launch
```

## 5. launch 文件说明

| 文件 | 角色 | 是否对外 |
| --- | --- | --- |
| `launch/greenhouse_mapping.launch` | 建图一键入口 | 是 |
| `launch/greenhouse_navigation.launch` | 导航默认入口（等价正向模式） | 是 |
| `launch/greenhouse_navigation_forward.launch` | 导航正向模式入口 | 是 |
| `launch/greenhouse_navigation_reverse.launch` | 导航反向模式入口 | 是 |
| `launch/system_greenhouse.launch` | 导航兼容别名 | 是 |
| `launch/system_greenhouse_with_route_planner.launch` | FAR + base autonomy 内部装配层 | 否 |
| `launch/system_greenhouse_base.launch` | 定位 + terrain + CMU local 内部装配层 | 否 |
| `launch/system_greenhouse_bag.launch` | bag 回放入口 | 否 |
| `launch/far_route_planner.launch` | FAR 规划子装配 | 否 |
| `launch/base_autonomy_cmu.launch` | CMU local + path follower 子装配 | 否 |
| `launch/localization_manager.launch` | 对外状态估计统一出口 | 否 |
| `launch/global_to_local_adapter.launch` | 兼容性 adapter 入口 | 否 |

## 6. 默认 topic 合同

### 对外关键输入

- `/goal_point`
- `/cloud_registered`
- `/terrain_map`
- `/terrain_map_ext`
- `/state_estimation`

### 对外关键输出

- `/state_estimation`
- `/way_point`
- `/navigation_boundary`
- `/path`
- `/speed`
- `/cmd_vel_stamped`
- `/cmd_vel`

## 7. 建图阶段建议入口

当前仓库里，建图优先使用一键入口：

```bash
roslaunch greenhouse_nav greenhouse_mapping.launch
```

如果只想单独拉起原始组件，才再使用：

```bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio mapping_mid360.launch
```

如果要在建图结束后导出 `scans.pcd`，需要先把：

```bash
src/FAST_LIO/config/mid360.yaml
```

中的：

```yaml
pcd_save:
  pcd_save_en: true
```

打开。

## 8. 主线设计约定

### 8.1 当前默认主线

当前主线是：

```text
FAR -> /way_point + /navigation_boundary -> CMU local
```

而不是：

```text
nav_msgs/Path -> adapter -> local planner
```

### 8.2 adapter 的定位

`global_to_local_adapter.py` 仍然保留，但它的用途已经收缩为：

- 兼容旧接口
- 做离线实验或替代方案验证

它**不是默认主线必经节点**。

## 9. 开发常用命令

### 9.1 编译

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 9.2 一键入口

```bash
roslaunch greenhouse_nav greenhouse_mapping.launch
roslaunch greenhouse_nav greenhouse_navigation.launch
```

### 9.3 只看导航 launch 节点展开

```bash
roslaunch greenhouse_nav greenhouse_navigation.launch --nodes
```

### 9.4 无硬件 smoke test

这个场景不是给操作层日常使用的，因此仍然保留在内部装配入口里。  
具体命令见：

- `../../.agentdocs/命令索引.md`

## 10. 相关脚本

### `scripts/localization_manager.py`

- 负责对齐定位输出接口
- 对外统一发布 `/state_estimation`
- 将 `camera_init -> body` 与固定安装外参 `body -> base_link` 合成为 `map -> base_link`
- 同时通过静态 TF 额外提供 `body_foot` / `robot_foot_init` 给建图与上层可视化使用

### `scripts/cmd_vel_stamped_bridge.py`

- 将局部规划输出的 `/cmd_vel_stamped` 转换为底盘侧常用的 `/cmd_vel`

### `scripts/global_to_local_adapter.py`

- 保留用于兼容和实验
- 默认主线关闭

## 11. 边界说明

- 本包是**装配层**，不是算法大包
- 规划、地形分析、定位与底盘驱动的主体实现分别仍在其他包中
- 若要替换某个算法模块，优先在装配层 launch 中替换，不要把大量逻辑堆进本包脚本里
