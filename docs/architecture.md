# 系统架构

## 数据流

整车采用 ROS 2 Jazzy。Mid-360 的 LiDAR/IMU 数据进入 FAST-LIO 前端，连续里程计驱动局部控制；共享地图核心提供 ikd-Tree 和 OctVox。后台回环通过 Scan Context、GICP 和 iSAM2 计算全局校正。

注册点云经坐标规范化与地形分析进入 FAR。默认由 CMU local planner 根据 FAR 航点生成局部路径，通用 follower 产生速度；可选 MPPI 使用 Nav2 FollowPath 与地形代价地图。两种模式共享航点任务、部署校验、命令适配与门控，底盘驱动负责串口协议。

| 职责 | ROS 包 |
| --- | --- |
| 驱动 | `livox_ros_driver2`、`dlrobot_robot` |
| 感知 | `greenhouse_cloud_normalizer`、`terrain_analysis` |
| 定位建图 | `fast_lio_super`、`slam_octvox`、`fast_lio_super_ros2`、`fast_livo` |
| 导航 | `greenhouse_mppi_navigation` 装配任务与两种控制模式；默认使用 `cmu_local_planner`、`cmu_path_follower_generic` |
| 控制 | `greenhouse_cmd_gate` |
| 系统装配 | `greenhouse_nav2_bringup` |

## 坐标与控制权

`map → odom → base_link → livox_frame` 的每条 TF 只有一个发布者。回环启用时拥有 `map → odom`，关闭时由 identity 发布者提供；FAST-LIO 拥有 `odom → base_link`，安装外参提供最后一条静态变换。回环校正保留局部里程计连续性。

CMU 发布 `/navigation/cmu_cmd`，MPPI 发布 `/navigation/mppi_cmd`。部署配置 `controllers` 只允许 `[cmu]` 或 `[mppi]`，默认模板选择 CMU。适配器独占 `/cmd_vel/nav`，门控独占 `/cmd_vel`；地图、TF、里程计、任务心跳或当前控制链输入失效时停止控制。

CMU 规划器和 follower 使用连续 `odom` 里程计，地形点云和 FAR 航点按消息时间转换到 `odom`；局部路径以 `base_link` 表达。两种模式共用单目标、多航点和取消接口。CMU 只提供位置到达控制；需要终点朝向时选择 MPPI。

当前 CMU 链依据活动 FAR route、waypoint 生命周期和 local Path 新鲜度拒绝陈旧输入，尚未向 local Path 传播显式 mission identity。时间戳条件不能证明跨话题延迟、重排或旧计算结果的任务归属。后续长期实机运行前，应实现贯穿规划与跟随链的任务 epoch/UUID，或可确认旧计算与缓存已失效的 reset 边界，并验证取消 A、启动 B 后晚到的 A 路径不能重新授权运动。

## 本地 CMU 来源

[CMU 官网](https://www.cmu-exploration.com/) 的 Autonomous Exploration 仓库目前位于 [HongbiaoZ/autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)，其父仓为 `jizhang-cmu/ground_based_autonomy_basic`。

| 本库包 | 上游组件 | 精确 fork 基准 |
| --- | --- | --- |
| `terrain_analysis` | Autonomous Exploration 的 `terrain_analysis` | 尚未确认 |
| `cmu_local_planner` | Autonomous Exploration 的 `local_planner` | 尚未确认 |
| `cmu_path_follower_generic` | Autonomous Exploration 的 `local_planner/pathFollower` | 尚未确认 |

外部 Go2 的固定 commit 仅管理扩展地形与 FAR 配套组件。本地 `paths/` 是规划器运行资产；生成器与现有 correspondence 数据的逐字复现关系尚未验证。

## 算法工作区

`fast_lio_super` 是 Noetic catkin 包，`fast_lio_super_ros2` 在 Jazzy 中编译相邻目录的共享核心。两种 ROS 分别构建，包名与运行接口由各自 manifest 和 launch 定义。

FAST-LIVO2 完整源码在 `src/localization/fast_livo` 中维护，使用独立 Noetic 工作区。其图像存储、地图预算和资源生命周期配置见 [FAST-LIVO2](../src/localization/fast_livo/README.md)。Livox 的 ROS 1 和 ROS 2 构建共用 `src/drivers/livox_ros_driver2`；外部 SDK、GTSAM、Sophus、vikit 及 Jazzy mapper 的固定来源与补丁统一位于 `third_party/`。

地图接口见 [SLAM_CORE_API](../src/localization/fast_lio_super/docs/SLAM_CORE_API.md)，回环参数与输出见 [CLOSED_LOOP](../src/localization/fast_lio_super/docs/CLOSED_LOOP.md)，部署与任务接口见 [导航说明](../src/navigation/greenhouse_mppi_navigation/README.md)。
