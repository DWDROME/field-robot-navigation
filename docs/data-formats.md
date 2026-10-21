# 数据格式与接口

> 范围：升级线的接口合同、部署输入和仿真数据。权威定义在配置文件与源码，这里只记语义和修改边界；TF 拓扑见 [architecture.md](architecture.md#坐标与控制权)。
> 核对：2026-10-21，静态阅读配置与源码；未重建镜像、未跑矩阵。

## 接口合同

[interfaces.yaml](../src/bringup/greenhouse_nav2_bringup/config/interfaces.yaml) 是话题、类型、owner 与控制权的唯一来源，代码和测试都引用它。

- `topics[].owners` 按 profile 指定发布者，一个话题只能有一个所有者。
- `command_contract`：`/cmd_vel` 只允许 `command_gate` 发布，输入源限定 `/cmd_vel/nav`、`/cmd_vel/teleop`、`/cmd_vel/test`。
- `tf`：每对父子变换只有一个发布者，新增 TF 先登记再实现。

常改的几个话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sensors/lidar/raw` | CustomMsg 或 PointCloud2 | 硬件为 CustomMsg，仿真为 PointCloud2 |
| `/sensors/imu/data` | Imu | 硬件来自 Livox，仿真来自桥接，不取自底盘串口 |
| `/localization/registered_cloud`、`/localization/odometry` | PointCloud2、Odometry | 定位输出 |
| `/perception/terrain_map` | PointCloud2 | 地形分析 |
| `/planning/local_path` | Path | 局部路径 |
| `/cmd_vel`、`/platform/cmd_vel` | TwistStamped | 门控输出、平台执行 |
| `/system/bringup_health` | DiagnosticArray | 启动健康检查 |

`/cmd_vel/nav`、`/cmd_vel/teleop`、`/cmd_vel/test`、`/planning/way_point`、`/robot_vgraph`、`/decoded_vgraph` 的类型与 owner 直接看合同文件。

## QoS

[qos.yaml](../src/bringup/greenhouse_nav2_bringup/config/qos.yaml) 固定每个话题的 `reliability`、`durability`、`history`、`depth`。传感器用 `best_effort`，控制用 `reliable` + `depth 1`，`/tf_static` 用 `transient_local`。

改深度要有实测积压与延迟证据；外部 writer 无法内省 history/depth 时记录限制，不填猜测值。

## Profile 与主机

[profiles.yaml](../src/bringup/greenhouse_nav2_bringup/config/profiles.yaml) 的字段含义：

- `components`：`owner`（compose/bringup/tools/external/experiment）与 `role`（sensor/localization/perception/planning/control/hardware/tf/interface）。
- `profiles`：`available`、`runtime_ready`、`runtime_ready_hosts`、`runtime_blocker`、`components`、`conflicts`。`runtime_ready_hosts` 为空表示未在任何主机就绪。
- `conflicts` 是互斥声明，启动时不得同时装配；改归属或互斥关系要同步装配测试。

[devices.yaml](../src/bringup/greenhouse_nav2_bringup/config/devices.yaml) 按主机声明 `use_sim_time`、`network_mode`、`required_devices`、`sensor_mode` 与 `qualification_boundary`。`nuc-amd64` 需要 `/dev/dlrobot` 和 `MID360_CONFIG_PATH`、`ROS_DOMAIN_ID`、`ROS_DISCOVERY_SERVER`、`FASTDDS_DEFAULT_PROFILES_FILE`；边界说明不要删。

## 部署配置

模板：[deployment-hardware.template.yaml](../src/navigation/greenhouse_mppi_navigation/config/deployment-hardware.template.yaml)。

| 字段 | 含义 |
| --- | --- |
| `mode` | `hardware` |
| `localization` | 定位来源，如 `fast_lio_super` |
| `controllers` | `[cmu]` 或 `[mppi]` |
| `loop_enabled` | 是否启用回环 |
| `mission` | `xy_tolerance`、`yaw_tolerance`、`waypoint_timeout`、`planning_timeout`、`path_timeout` |
| `sensor` | `network`、`driver_config`、`imu_from_lidar_T/R`、`base_from_lidar_xyzrpy` |
| `chassis` | 运动模型、设备、波特率、footprint、速度/加速度限值、急停与使能话题 |

模板里的 `null` 是刻意的：没填全就拒绝硬件激活，不会用默认值顶替。单位与字段名不要改。

## 雷达 JSON

[MID360_config.example.json](../src/bringup/greenhouse_nav2_bringup/config/MID360_config.example.json) 与 main 的 `MID360_config.json` 同构：网点端口、上位机 IP、雷达 IP、`pcl_data_type`、`pattern_mode` 和 `extrinsic_parameter`（rad / m）。复制为本地配置后改 IP，不要把现场地址提交回仓库。

## 仿真数据

**地面任务** [ground_sites.yaml](../src/bringup/greenhouse_nav2_bringup/config/simulation/tasks/ground_sites.yaml)：按场地给 `spawn_world_xy` 与 `cases[]`；每项含 `name`、`expectation`（`reach` 或 `reach_or_refuse`）、`geometry`、`waypoints`（传感器定位后的地图坐标，yaw 0）。

**空中航线** [x500_sites.yaml](../src/bringup/greenhouse_sim_air/config/x500_sites.yaml)：`spawn`（x/y/z/yaw）、`cruise_altitude`、`waypoints {x,y,z}`，坐标是 PX4 本地 NED 语义，由任务节点转换。改航线后重跑 `inspect-simulation-geometry.py`，净空报告与航线哈希要一致。

**机器人与桥接**：`simulation/robot.yaml`、`robots/*.yaml` 描述平台与命名空间，`sensor-bridge-*.yaml` 是 ros_gz 桥接；改几何或桥接话题会影响接口合同与 TF 归属。

## 仿真证据

- 入口 `simulation_matrix.launch.py`，参数 `world`、`robot`、`headless`、`seed`、`task`、`output`。
- 输出 `matrix_results.json` / `matrix_results.csv`，单次运行另有 JSON 与日志；截图带 sidecar（运行编号、场景、位姿、仿真时间、状态）。
- `--resume` 只续跑缺失项；实现或配置哈希变化时拒绝复用。
- 截图不替代逐张视觉复核；失败记录不删。

## 串口

11 字节命令、24 字节遥测，`0x7B`/`0x7D`，异或校验；`PROTOCOL_25` 必须为 0。遥测 `8..19` 是控制诊断分页，`Telemetry.diagnostics` 原样保存，不换算 IMU、不发布 `/imu`。逐字节表见[固件协议](../firmware/tr800_f407/docs/TR800_通信协议.md)。

改动同步固件 `HARDWARE/usartx.h`、本线 `src/drivers/dlrobot_robot/include/dlrobot_robot/protocol.hpp` 与 main 的 `src/dlrobot_robot/include/dlrobot_robot.h`。

## 依赖与命名

- `third_party/*.repos`：`repositories: <name>: {type: git, url, version}`，`version` 写完整 commit；补丁为 unified diff。
- `docker/config/*.packages` 是纯文本 APT 清单；`docker/Dockerfile` 固定基础镜像 digest，改镜像输入要记录 digest。
- 新增话题按 `/<域>/<对象>` 命名并先登记到 `interfaces.yaml`；同一话题单一 owner、同一 TF 单一发布者。
- 配置改动后跑对应合同测试（`test_contracts.py`、`test_launch_contract.py`）再跑入口冒烟。
