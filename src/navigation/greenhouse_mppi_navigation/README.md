# FAR、地形与履带导航

本包提供 ROS 2 Jazzy 的整车控制装配。部署模板默认 `controllers: [cmu]`：FAR 航点 → CMU local planner → 通用 follower → 命令适配器 → `greenhouse_cmd_gate` → 底盘。设置 `controllers: [mppi]` 时使用 FAR 完整图路径与 Nav2 FollowPath/MPPI。两种模式共用以下部署、任务及取消入口，配置只允许选择一个控制器；运行中重复命令拥有者使输出归零。

## 部署输入和启动

先复制 `config/deployment-hardware.template.yaml` 到部署目录，填写实际 Mid-360 主机/雷达 IP、网络接口、driver JSON 路径、IMU/LiDAR 外参、base/LiDAR 安装变换、稳定底盘设备路径、波特率、footprint、运动限值，以及 Bool 急停/使能接口。急停 true 表示禁止运动；急停消息须持续更新，失联也禁止运动。使能接口由实际平台执行，源码没有验证其硬件效力。

```bash
ros2 run greenhouse_mppi_navigation validate_deployment /absolute/deployment.yaml --resources
ros2 launch greenhouse_nav2_bringup closed_loop.launch.py deployment:=/absolute/deployment.yaml map_backend:=octvox
```

缺失或非法输入、网络地址不匹配、driver 重复施加外参、设备不存在等都会在启动动作创建前拒绝。`deployment-sim.yaml` 仅是声明过的协议样例，需要外部仿真时钟及传感器；不能直接当实车参数。软件配置完备产生 `runtime_ready=true`，不会产生 `hardware_qualified=true`。

## 单目标、多航点和取消

使用标准 `nav2_msgs/action/FollowWaypoints`，单目标就是长度为 1 的列表。仅支持一次顺序执行，`number_of_loops=0`、`goal_index=0`；每个 PoseStamped 的 frame 必须是 map，四元数必须归一化。提交任务前确认 `/navigation/data_ready` 为 true。

```bash
ros2 action send_goal /navigation/follow_waypoints nav2_msgs/action/FollowWaypoints "{poses: [{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 4.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}]}" --feedback
ros2 topic echo /navigation/mission_status
```

action 客户端可取消返回的 goal handle；也可以通过标准取消服务取消当前任务：

```bash
ros2 service call /navigation/follow_waypoints/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

任务状态包括 planning、controlling、completed、failed、timeout、cancelled。部署文件的可选 `mission` 配置提供 `xy_tolerance`（默认 0.3 m）、`yaw_tolerance`（默认 π）、`waypoint_timeout`（默认 120 s）、`planning_timeout`（默认 15 s）和 `path_timeout`（默认 1 s）。MPPI 模式将 XY/yaw 容差同时应用到 mission 和 controller goal checker；需要终点朝向时选择 MPPI 并降低 yaw_tolerance。到达由最新 map 位姿判定，控制 action 的成功结果不单独推进航点。失败不会跳过航点。

## 路径与消息合同

FAR 补丁从 `GraphPlanner::PathToGoal` 的 `NodePtrStack` 发布 `/planning/global_path` (`nav_msgs/msg/Path`)，保留完整图节点顺序及切线朝向。它没有从 `/way_point` 或 Marker 猜测路径。`/planning/route_status` 是 String JSON，包含 `goal_stamp_ns`、`plan_stamp_ns`、`status`。Path 时间为 plan 时间；目标 PointStamped 时间是唯一任务 token。两条消息以 plan 时间配对，最多缓存 8 条。

新目标先使旧 FAR 路径失效；`/planning/cancel_goal` (Int64) 只取消对应目标 token。ready/reached 状态携带规划路径，planning/not-ready/unreachable/stale-input/cancelled 不授权继续控制。路径默认 1 s 过期；无路径或过期会终止任务，在 MPPI 模式取消 FollowPath，并请求零速度。FAR route 身份与 MPPI action 回调按 generation/token 校验。

CMU local Path 尚未携带显式 mission identity。当前实现根据活动 route、waypoint 生命周期和 Path 新鲜度拒绝陈旧输入，但不能用时间戳证明所有跨任务 Path 的因果归属。贯穿 CMU 规划与跟随链的 epoch/UUID，或可确认清除旧计算的 reset 边界，是长期实机运行前的后续工程项；不能把当前防护表述为完整任务隔离保证。

MPPI 发送 FollowPath 时保留 FAR 的全部坐标，只将最后一项姿态设为当前任务请求的终点姿态；终点朝向由 MPPI goal checker 和 mission 的相同 yaw 容差共同约束。CMU 使用 FAR 航点生成局部路径，仅支持位置到达，`yaw_tolerance` 必须为 π；需要终点朝向时选择 MPPI。

## 地形、TF 和命令

CMU 模式将地形和 FAR 航点按消息时间转换到 `odom`，规划器和 follower 共用连续本地里程计。路径库保留原有数据，按 footprint 外接圆半径与源码中的 0.45 m searchRadius 比例设置路径尺度，关闭随速度缩小路径尺度。现有 correspondence 数据与生成器的对应关系尚未验证，此配置不代表已验证实车碰撞余量。只有当前任务的 FAR 路径、后续航点与局部路径均新鲜时才允许输出；航点或局部路径过期时终止任务并停车。

`terrain_grid` 用真实消息时间 TF 将地形点云转换到 odom，保留 intensity 的相对高度语义。高度大于 `obstacle_height` 或低于负 `drop_height` 的点形成障碍；同格障碍优先。每帧替换网格，未观测为 -1，旧障碍不累积。TerrainLayer 将未知区保守视为不可通行，后续 inflation 处理 footprint 周边代价。过期层为非 current 且填充致命代价。

MPPI 用 DiffDrive 模型、部署 footprint/速度/加速度和显式 critic，controller_server 与 local_costmap 通过 lifecycle manager 激活。MPPI 的 `enable_stamped_cmd_vel=true`，输出只在 `/navigation/mppi_cmd`。命令适配器检查连续 odometry、terrain、TF、任务心跳、源时间和 receipt 时间，使用 steady watchdog；取消后重新使能会设置时间 fence，旧命令不能恢复运动。

CMU 的原始命令为 `/navigation/cmu_cmd`，MPPI 为 `/navigation/mppi_cmd`。适配器独占 `/cmd_vel/nav` (`TwistStamped`, base_link)，门控独占 `/cmd_vel`，`dlrobot_robot` 独占串口。最终门控还限制速度/加速度并处理看门狗。局部控制读取 odom，FAR/地形读取 map；注册点云经 `greenhouse_cloud_normalizer` 实际转换。回环和 identity 的 map→odom 发布者互斥。

## 构建

构建见 [构建说明](../../../docs/build.md)，Docker 目标为 `production-runtime`，交付平台为 amd64。外部 Git 依赖固定于 `third_party`，Livox 驱动由本库维护；CMU 与 MPPI 安装在同一镜像中，APT 依赖按包名管理。

`docker/compose.yaml` 提供闭环运行服务，设备及配置通过部署目录显式挂载。包内 `test/` 提供导航合同、launch 装配和节点接口检查源码。
