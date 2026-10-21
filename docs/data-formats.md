# 数据格式

> 范围：跨进程、跨设备、跨版本传递的数据。TF 拓扑见 [architecture.md](architecture.md#原版数据流)，串口完整字节表见 [TR800_通信协议.md](../firmware/tr800_f407/docs/TR800_通信协议.md)，本文只记约束与索引。
> 核对：2026-10-21，按源码与配置静态阅读；未抓包、未解析实测帧。

## 单位与坐标

长度 m、角度 rad、速度 m/s、角速度 rad/s，时间为 ROS 时间。点云与地图按 PCL 右手系，地图原点由建图时确定，换场地要重新确认。定位链 `map → camera_init → body → base_link`、底盘 `/odom` 的坐标声明见架构文档。

## 地图

**三维 PCD**

- 生成：`save_map:=true` 打开 `pcd_save`，正常 Ctrl+C 后由 `PCDWriter::writeBinary` 写 `src/FAST_LIO/PCD/scans.pcd`，点类型为 `PointXYZINormal`。
- 使用：`map:=/absolute/map.pcd`；`pcd_to_pointcloud` 以 `frame_id=map` 发布 `/map`，周期 5 s。
- 约束：绝对路径，坐标系与现场一致；`*.pcd`、`*.bag` 不入库。

**二维辅助图**

`publish_2d_map:=true map_2d:=/absolute/map.yaml`，按 `map_server` 格式（YAML + PGM/PNG）。YAML 与图片必须同源。

## 雷达配置

`src/livox_ros_driver2/config/MID360_config.json`，运行时用 `user_config_path:=` 覆盖。

| 字段 | 说明 |
| --- | --- |
| `lidar_summary_info.lidar_type` | MID360 固定 `8` |
| `MID360.lidar_net_info.*_port` | 雷达侧端口：命令 56100、推送 56200、点云 56300、IMU 56400、日志 56500 |
| `MID360.host_net_info.*_ip`、`*_port` | 上位机网卡地址；端口默认雷达端口 +1 |
| `lidar_configs[].ip` | 雷达地址 |
| `lidar_configs[].pcl_data_type`、`pattern_mode` | 当前 `1`、`0` |
| `lidar_configs[].extrinsic_parameter` | 安装外参：`roll/pitch/yaw` 为 rad，`x/y/z` 为 m |

## 算法参数

FAST-LIO 与 FAST-LIO_LOCALIZATION 共用结构，文件为各自 `config/mid360.yaml`：

| 分组 | 主要键 |
| --- | --- |
| `common` | `lid_topic`、`imu_topic`、`time_sync_en`、`time_offset_lidar_to_imu` |
| `preprocess` | `lidar_type`（1 Livox / 2 Velodyne / 3 Ouster）、`scan_line`、`blind` |
| `mapping` | `acc_cov`、`gyr_cov`、`b_acc_cov`、`b_gyr_cov`、`fov_degree`、`det_range`、`extrinsic_T/R`、`extrinsic_est_en` |
| `publish` | `path_en`、`scan_publish_en`、`dense_publish_en` |
| `pcd_save` | `pcd_save_en`、`interval`（`-1` 为单文件） |

MoveBase3D 参数在 `global_planner_3d/config/move_base_3d/`：`move_base_global.yaml`（地图范围、分辨率、规划权重）、`move_base_global_obs.yaml`（障碍）、`move_base_local.yaml`（局部控制）；地势参数在 `rover_gpr/config/hyperparam.txt`。字段说明以文件内注释为准，改值同时改注释。

## 话题与消息

跨包契约如下，完整列表以源码的 `advertise`/`subscribe` 为准。

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/livox/lidar`、`/livox/imu` | CustomMsg/PointCloud2、Imu | 驱动 → LIO |
| `/cloud_registered`、`/cloud_registered_body` | PointCloud2 | LIO 点云（`camera_init` / `body`） |
| `/Odometry` | Odometry | LIO 里程计（`camera_init → body`） |
| `/map` | PointCloud2 | 先验地图（`map`） |
| `/localization` | Odometry | 融合定位（`map → body`） |
| `/output_filter` | PointCloud2 | 过滤后的实时障碍 |
| `/cmd_vel` | Twist | 规划 → 底盘 |
| `/odom`、`/PowerVoltage` | Odometry、Float32 | 底盘轮速积分与电压；无串口 IMU |

消息包：`dl_msgs`（`Velocities`、`Imu`、底盘外设）、`rover_msgs`（目标与状态）、`livox_ros_driver2`（`CustomMsg`、`CustomPoint`）、FAST_LIO 的 `Pose6D`。

## 串口

11 字节命令、24 字节遥测，帧头 `0x7B`、帧尾 `0x7D`，逐字节异或校验；命令为 `int16_be`，发送前乘 1000。`PROTOCOL_25` 必须为 `0`。遥测 `8..19` 字节当前是控制诊断（`CONTROL_TELEMETRY_DEBUG=1`），驱动原样保留、不换算 IMU、不发布 `/imu`；导航惯性输入来自 Livox。

协议字段改动要同步固件 `HARDWARE/usartx.h`、本线 `src/dlrobot_robot/include/dlrobot_robot.h` 与升级线驱动。

## 固件产物与依赖

- 固件产物 `dlrobot.elf/hex/bin` 在 `build/tr800-st-preset-win/`，preset 为 `tr800-st`；产物不入库。
- 依赖固定用 `third_party/common.repos`：`repositories: <name>: {type: git, url, version}`，`version` 写完整 commit；补丁按文件名顺序 `git apply`。

## 改动前检查

1. 地图存在、是绝对路径、坐标系正确。
2. 雷达 JSON 的 host IP 是本机网卡地址。
3. 外参有改动就重新标定；`extrinsic_est_en` 与固定外参不要混用。
4. 串口设备名、`dialout` 权限、波特率与固件一致。
5. 协议字段改动已同步固件与两条线的驱动。
