# Greenhouse Navigation Workspace

ROS1 Noetic `catkin` 工作空间，用于温室场景下的激光定位、地形分析、全局/局部规划与底盘执行联调。

## 1. 当前状态

- 已收口两个用户入口：
  - 建图：`roslaunch greenhouse_nav greenhouse_mapping.launch`
  - 导航：`roslaunch greenhouse_nav greenhouse_navigation.launch`
- 兼容旧命令：`roslaunch greenhouse_nav system_greenhouse.launch`
- 已完成：
  - 新工作空间迁移
  - 全量 `catkin_make` 编译
  - 无硬件 smoke test
- 尚未默认宣称：
  - 实机闭环已稳定可用
  - 所有现场传感器组合已跑通

## 2. 主线组成

当前温室主线默认由以下模块构成：

- `livox_ros_driver2`：Mid360 雷达接入
- `FAST_LIO`：激光里程计 / 建图
- `FAST_LIO_LOCALIZATION`：全局重定位
- `greenhouse_nav`：主线集成、状态发布与桥接
- `terrain_analysis`：局部地形分析
- `terrain_analysis_ext`：扩展地形分析
- `far_planner`：全局 / 路线规划
- `local_planner`：CMU local planner + path follower
- `dlrobot_robot`：底盘串口执行

## 2.1 传感器安装姿态约定

- 当前 Mid360 安装不是水平，而是**向前下倾 27°**
- 这在 ROS 坐标语义里应理解为：**绕 Y 轴（pitch）旋转**
- 当前采用 **方案 A**：保留 FAST-LIO 原生 `camera_init -> body` 不动
- 额外发布水平辅助帧：
  - `body -> body_foot`
  - `camera_init -> robot_foot_init`
- 对现有导航兼容仍保留 `body -> base_link`
- 当前前下倾 27° 的补偿统一按：**`pitch = -27° = -0.471239 rad`**

这层安装外参现在不再塞进：

- `livox_ros_driver2` 的 `extrinsic_parameter`
- `FAST_LIO/config/mid360.yaml` 的 `extrinsic_R/T`

而是收敛到：

- `greenhouse_nav/launch/system_greenhouse_base.launch`
- `greenhouse_nav/localization_manager.py`

目的有两个：  
1. 让建图/可视化优先基于 `robot_foot_init` / `body_foot` 这套水平辅助帧  
2. 保持 `/state_estimation` 与 `base_link` 兼容现有导航下游

补充说明：

- `FAST-LIO` 默认发布的是 `camera_init -> body`
- 若启动瞬间传感器本体本来就是前下倾，`camera_init` 也会继承这份初始姿态
- 所以在 RViz 里如果 `Fixed Frame = camera_init`，`/cloud_registered` 看起来仍可能是倾斜的
- 当前建图入口默认改为 **`Fixed Frame = robot_foot_init`**
- 这样看到的是“水平辅助世界帧”，而不是 FAST-LIO 原生初始姿态帧

## 3. 默认数据流

```text
Mid360
  -> livox_ros_driver2
  -> FAST_LIO / FAST_LIO_LOCALIZATION
  -> greenhouse_nav/localization_manager
  -> /state_estimation (map -> base_link)

/cloud_registered + /terrain_map + /terrain_map_ext + /goal_point
  -> far_planner
  -> /way_point + /navigation_boundary

/state_estimation + /way_point + /navigation_boundary + /terrain_map
  -> localPlanner + pathFollower
  -> /path + /cmd_vel_stamped
  -> cmd_vel_stamped_bridge
  -> /cmd_vel
  -> dlrobot_robot
```

## 4. 快速开始

### 4.1 编译

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 4.2 启动建图

```bash
roslaunch greenhouse_nav greenhouse_mapping.launch
```

说明：

- 该入口现在默认打开 `greenhouse_nav/rviz/greenhouse_mapping_base_link.rviz`
- RViz 默认 `Fixed Frame = robot_foot_init`
- 目的不是改 FAST-LIO 内部世界帧，而是在其之上补一层水平辅助帧，避免被 `camera_init` 的初始倾斜语义误导

### 4.3 启动导航

```bash
roslaunch greenhouse_nav greenhouse_navigation.launch
```

### 4.4 仅检查导航 launch 装配关系

```bash
roslaunch greenhouse_nav greenhouse_navigation.launch --nodes
```

## 5. 标准工作流：从建图到导航

### 5.1 这套默认是否需要预建图

需要。  
当前默认导航主线 `greenhouse_navigation.launch` 会同时启用：

- `FAST_LIO_LOCALIZATION`
- 地图发布 `/map`
- 全局重定位

因此它默认假设你已经有一张可用的全局点云地图。

### 5.2 需要准备什么

#### 必需物

- Mid360 LiDAR
- Mid360 IMU（同设备自带）
- ROS1 Noetic 工作空间
- 一张预建好的全局点云地图：`PCD`
- 启动时的一次粗初值：`/initialpose`

#### 当前默认地图位置

```bash
src/FAST_LIO/PCD/scans.pcd
```

### 5.3 建图阶段

当前仓库已经补上“用户级一键建图入口”。

#### 第一步：打开地图保存

建图前，将：

```yaml
src/FAST_LIO/config/mid360.yaml
```

中的：

```yaml
pcd_save:
  pcd_save_en: true
```

打开。  
建图完成后，如不想持续保存点云，可再改回 `false`。

#### 第二步：启动建图入口

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch greenhouse_nav greenhouse_mapping.launch
```

#### 第三步：采一圈完整环境

建议动作：

- 起点静止几秒
- 缓慢前进
- 覆盖整个温室目标区域
- 尽量减少急转和高速运动
- 尽量避开大规模动态干扰
- 回到已知区域闭环更稳

#### 第四步：结束建图并取地图

停止 `FAST_LIO` 后，地图会保存到：

```bash
src/FAST_LIO/PCD/scans.pcd
```

### 5.4 导航阶段

#### 第一步：确认地图文件存在

```bash
ls src/FAST_LIO/PCD/scans.pcd
```

#### 第二步：启动默认导航主线

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch greenhouse_nav greenhouse_navigation.launch
```

#### 第三步：给初始位姿

`FAST_LIO_LOCALIZATION` 默认会等待 `/initialpose`。  
当前仓库已带脚本：

```bash
rosrun fast_lio_localization publish_initial_pose.py x y z yaw pitch roll
```

例如：

```bash
rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
```

> 注意：这里的数值应该是**地图坐标系 `map` 下的粗初值**。  
> 不需要特别精，但不能离真实位置太远。

#### 第四步：发送导航目标

例如直接打一发目标点：

```bash
rostopic pub -1 /goal_point geometry_msgs/PointStamped \
'{header: {frame_id: "map"}, point: {x: 3.0, y: 0.0, z: 0.0}}'
```

### 5.5 运行时你应该关注什么

关键 topic：

- `/state_estimation`
- `/cloud_registered`
- `/terrain_map`
- `/terrain_map_ext`
- `/way_point`
- `/navigation_boundary`
- `/path`
- `/cmd_vel_stamped`
- `/cmd_vel`

## 6. 入口约定

### 对外用户入口

- 建图：`src/greenhouse_nav/launch/greenhouse_mapping.launch`
- 导航：`src/greenhouse_nav/launch/greenhouse_navigation.launch`

其中导航入口默认已经固化：

- Livox
- FAST-LIO
- global localization
- terrain analysis
- FAR
- CMU local planner
- `cmd_vel` 桥接
- 底盘串口

### 兼容入口

- `src/greenhouse_nav/launch/system_greenhouse.launch`

该文件仍可继续使用，但现在只是 `greenhouse_navigation.launch` 的兼容别名。

### 内部装配入口

以下 launch 保留给回放、smoke test 和模块级调试使用：

- `src/greenhouse_nav/launch/system_greenhouse_with_route_planner.launch`
- `src/greenhouse_nav/launch/system_greenhouse_base.launch`
- `src/greenhouse_nav/launch/system_greenhouse_bag.launch`

## 7. 推荐工作区目录

本仓库可以直接作为本地 `catkin` 工作空间使用。  
远端部署时，当前建议工作区目录为：

```bash
~/greenhouse_test
```

常用初始化：

```bash
cd ~/greenhouse_test
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 8. 文档索引

- 主线方案：`".agentdocs/计划书.md"`
- 执行拆解：`".agentdocs/执行计划书.md"`
- 命令索引：`".agentdocs/命令索引.md"`
- 包级说明：`"src/greenhouse_nav/README.md"`

## 9. 当前边界

- 该仓库里仍保留历史包与实验材料，但**当前温室主线入口已经收口到 `greenhouse_nav`**
- `global_to_local_adapter` 仍保留，但**默认主线不启用**
- 若要做 bag 回放、无硬件验证或模块替换，不要直接改 `system_greenhouse.launch`，优先使用内部装配入口
