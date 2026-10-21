# 数据格式与接口规范（upgrade/ros2）

本文件汇总升级线上跨节点、跨容器、跨版本传递的数据格式与合同。权威定义在配置文件和源码里，本文给出入口、字段语义与修改边界。原版 ROS 1 线的对应说明见 `main` 分支的 `docs/data-formats.md`。

## 1. 通用约定

- 配置统一 YAML，带 `schema_version: 1`；新增或改结构时同步升版本并在评审中说明。
- 单位：长度 m、角度 rad、速度 m/s、角速度 rad/s；时间使用 ROS 时间。
- 坐标系：`map → odom → base_link`，传感器挂在 `base_link` 下（如 `livox_frame`）。所有权见第 2 节。

## 2. 接口合同 `config/interfaces.yaml`

这是本线最重要的一份数据规范：话题名、消息类型、按 profile 的发布者、以及控制权归属都写在这里，代码与测试都引用它。

- `topics`：每个话题的类型（可多类型）与 `owners`（哪个 profile 的哪个组件发布）。
- `command_contract`：最终速度话题 `/cmd_vel` 只允许 `command_gate` 一个发布者，输入源限定 `/cmd_vel/nav`、`/cmd_vel/teleop`、`/cmd_vel/test`。
- `tf`：每个 profile 的 TF 链与发布者，禁止多节点同时发布同一对父子变换。

关键话题：

| 话题 | 类型 | 发布者/说明 |
| --- | --- | --- |
| `/sensors/lidar/raw` | `livox_ros_driver2/msg/CustomMsg` 或 `sensor_msgs/msg/PointCloud2` | 驱动；硬件为 CustomMsg，仿真为 PointCloud2 |
| `/sensors/imu/data` | `sensor_msgs/msg/Imu` | 驱动 |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | 底盘 |
| `/localization/registered_cloud` | `sensor_msgs/msg/PointCloud2` | FAST-LIO 或 PCD 定位 |
| `/localization/odometry` | `nav_msgs/msg/Odometry` | 定位 |
| `/localization/global_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | PCD 定位 |
| `/perception/terrain_map` | `sensor_msgs/msg/PointCloud2` | 地形分析 |
| `/planning/local_path` | `nav_msgs/msg/Path` | 局部规划 |
| `/planning/way_point` | `geometry_msgs/msg/PointStamped` | FAR |
| `/robot_vgraph`、`/decoded_vgraph` | `visibility_graph_msg/msg/Graph` | FAR / 图解码 |
| `/cmd_vel/nav`、`/cmd_vel/teleop`、`/cmd_vel/test` | `geometry_msgs/msg/TwistStamped` | 三个输入源 |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 门控唯一输出 |
| `/platform/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 平台执行层 |
| `/system/bringup_health` | `diagnostic_msgs/msg/DiagnosticArray` | 启动健康检查 |

## 3. QoS 合同 `config/qos.yaml`

每个话题的 `reliability`、`durability`、`history`、`depth` 都在这里固定，例如传感器用 `best_effort`、控制用 `reliable` + `depth 1`、`/tf_static` 用 `transient_local`。修改深度必须有实测积压与延迟证据，不能凭感觉调大；外部 writer 的 history/depth 无法内省时记录限制，不伪造数值。

## 4. Profile 规范 `config/profiles.yaml`

- `components`：组件清单，每个组件标注 `owner`（compose/bringup/tools/external/experiment）与 `role`（sensor/localization/perception/planning/control/hardware/tf/interface）。
- `profiles`：可用性合同。字段包括 `role`、`available`、`runtime_ready`、`runtime_ready_hosts`、`runtime_blocker`、`components`、`conflicts`。
- `runtime_ready_hosts` 为空表示未在任何主机就绪；`conflicts` 声明互斥 profile，启动时不得同时装配。

修改组件归属、增加 profile 或改变互斥关系时，必须同时改本文件与对应的启动装配测试。

## 5. 主机与设备 `config/devices.yaml`

按主机定义 `use_sim_time`、`network_mode`、`required_devices`、`sensor_mode` 与 `qualification_boundary`。`nuc-amd64` 要求 `/dev/dlrobot`、`MID360_CONFIG_PATH`、`ROS_DOMAIN_ID`、`ROS_DISCOVERY_SERVER`、`FASTDDS_DEFAULT_PROFILES_FILE`。边界说明用于限定“这台机器上验证过什么”，不要删除。

## 6. 部署配置（硬件）

模板：`src/navigation/greenhouse_mppi_navigation/config/deployment-hardware.template.yaml`。

| 字段 | 含义 |
| --- | --- |
| `mode` | `hardware` |
| `localization` | 定位来源，如 `fast_lio_super` |
| `controllers` | `[cmu]` 或 `[mppi]`，二选一 |
| `loop_enabled` | 是否启用回环 |
| `mission` | `xy_tolerance`、`yaw_tolerance`、`waypoint_timeout`、`planning_timeout`、`path_timeout` |
| `sensor` | `network.interface/host_ip/lidar_ip`、`driver_config`、`imu_from_lidar_T/R`、`base_from_lidar_xyzrpy` |
| `chassis` | 运动模型、设备、波特率、footprint、速度/加速度限值、急停与使能话题 |

模板中的 `null` 是刻意的：未填全就拒绝硬件激活，而不是用默认值悄悄跑起来。填写时保持单位与字段名不变。

## 7. 雷达配置（JSON）

`config/MID360_config.example.json` 与 main 的 `MID360_config.json` 同构：`lidar_summary_info.lidar_type=8`、`MID360.lidar_net_info` 端口组、`MID360.host_net_info` 上位机 IP/端口、`lidar_configs[]` 的雷达 IP、`pcl_data_type`、`pattern_mode` 与 `extrinsic_parameter`（`roll/pitch/yaw` rad、`x/y/z` m）。复制为本地配置后改 IP，不要把现场地址提交回仓库。

## 8. 仿真数据

### 8.1 地面任务 `config/simulation/tasks/ground_sites.yaml`

- 以场地为键：`maize_field`、`orchard`、`pipeline`。
- `spawn_world_xy`：出生点；`cases[]` 每项含 `name`、`expectation`（`reach` 或 `reach_or_refuse`）、`geometry`（几何依据）与 `waypoints`（传感器定位后的地图坐标，yaw 0）。
- 判定规则：普通控制器错误不算“正确拒绝”；危险用例只有在同一轮正常用例成功后才具备拒绝验收前提。

### 8.2 空中航线 `config/x500_sites.yaml`

按场地给 `spawn`（x/y/z/yaw）、`cruise_altitude` 与 `waypoints {x,y,z}`；坐标是 PX4 本地 NED 语义（x 北、y 东、z 高度），由任务节点转换。航线改动后必须重跑 `inspect-simulation-geometry.py`，净空报告与航线哈希要一致。

### 8.3 机器人与桥接

`config/simulation/robot.yaml` 与 `robots/*.yaml` 描述 Clearpath 平台、附件与命名空间；`sensor-bridge-*.yaml` 是 ros_gz 桥接配置。修改机器人几何或桥接话题会影响接口合同与 TF 归属，需同步回归。

## 9. 仿真运行与证据

- 入口：`simulation_matrix.launch.py`，参数 `world`、`robot`、`headless`、`seed`、`task`、`output`。
- 结果：输出目录下 `matrix_results.json` / `matrix_results.csv`，单次运行另有 JSON 与日志；截图带 JSON sidecar（运行编号、world/robot、相机位姿、仿真时间、测试状态）。
- 恢复：`--resume` 只续跑缺失项；实现或配置哈希变化时拒绝复用，需换输出目录。
- 媒体：PNG 原图入 `docs/media/simulation/`，JPG 供 GitHub；截图不替代逐张视觉复核。

## 10. 串口协议（与 main 共用固件）

- 11 字节命令帧、24 字节遥测帧，帧头 `0x7B`、帧尾 `0x7D`、异或校验；命令为 `int16_be` 且发送前乘 1000。
- `PROTOCOL_25` 必须为 `0`；当前 `TR800_MOTOR_TELEMETRY_DEBUG=1` 时遥测第 8–19 字节是诊断字段，不是 IMU。
- 改动需同步固件 `HARDWARE/usartx.h`、本线 `src/drivers/dlrobot_robot/include/dlrobot_robot/protocol.hpp` 与 main 的 `src/dlrobot_robot/include/dlrobot_robot.h`。
- 完整字节表见 `firmware/tr800_f407/docs/TR800_通信协议.md`。

## 11. 依赖与镜像输入

- `third_party/*.repos`：`repositories: <name>: {type: git, url, version}`，`version` 写完整 commit；补丁为 unified diff。
- `docker/config/*.packages`：纯文本 APT 包清单，按构建/运行/仿真分文件。
- `docker/Dockerfile` 固定基础镜像 digest；Compose 文件固定服务、卷与资源限制。改镜像输入要记录 digest 与构建参数。

## 12. 命名与修改边界

- 话题按 `/<域>/<对象>` 命名（`sensors`、`localization`、`perception`、`planning`、`cmd_vel`、`platform`、`system`），新增话题先登记到 `interfaces.yaml`。
- 同一话题只能有一个所有者；同一 TF 只能有一个发布者。
- 配置改动后至少跑对应的合同测试（`test_contracts.py`、`test_launch_contract.py`），再跑入口冒烟。
- 仿真结论必须区分“接口回归通过”和“任务通过”，不能互相替代。
