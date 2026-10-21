# 数据格式与接口规范

本文件汇总 main 分支上“会跨进程、跨设备或跨版本传递的数据”分别是什么格式、定义在哪、有什么约束。字段细节以源码和配置文件为准；本文负责给出索引和边界，避免接手时重复翻查。

## 1. 单位与坐标系

- 长度米（m），角度弧度（rad），时间为 ROS 时间；速度 m/s，角速度 rad/s。
- 点云统一使用 PCL/ROS 的右手坐标系；地图坐标原点由建图时确定，切换场地后必须重新确认。
- 主要坐标系：

| 坐标系 | 含义 | 发布者 |
| --- | --- | --- |
| `map` | 先验地图坐标系 | 地图点云与 `transform_fusion` |
| `camera_init` | FAST-LIO 建图起点 | LIO 里程计 |
| `body` | 雷达/IMU 机体 | LIO 里程计 |
| `odom_combined` → `base_footprint` | 底盘里程计 | `dlrobot_robot` |
| `imu_link` | 底盘 IMU | `dlrobot_robot` |

`transform_fusion` 把 `map` 与 `camera_init` 连起来，并发布 `map → body` 的 `/localization`；底盘里程计链独立发布。

## 2. 地图数据

### 2.1 三维点云地图（PCD）

- 生成：FAST-LIO 的 `pcd_save` 在正常退出时用 `pcl::PCDWriter::writeBinary` 写 `src/FAST_LIO/PCD/scans.pcd`；`pcd_save_en` 默认关闭，建图模式由 `save_map:=true` 打开。
- 格式：PCL PCD 二进制，字段为 FAST-LIO 内部点类型（`x/y/z/intensity`，附法向与曲率）；读取端是通用 PCL，其他工具导出的 ASCII PCD 只要含 `x/y/z` 也能用。
- 使用：`map:=/absolute/map.pcd`。`pcd_to_pointcloud` 以 `frame_id=map`、每 5 秒重发一次，话题 `/map`。
- 约束：PCD 与录包都不入库（`.gitignore` 忽略 `*.pcd`、`*.bag`）；路径必须是绝对路径；坐标系必须与现场一致，否则定位会整体偏移。

### 2.2 二维辅助地图

- 需要旧版二维输出时传 `publish_2d_map:=true map_2d:=/absolute/map.yaml`。
- 格式为 ROS `map_server` 标准：YAML 描述 `image`、`resolution`、`origin`、`negate`、`occupied_thresh`、`free_thresh`，图片为 PGM/PNG。
- 约束：YAML 与图片必须来自同一张地图；坐标系与三维地图一致。

## 3. 雷达配置（JSON）

- 默认路径 `src/livox_ros_driver2/config/MID360_config.json`，运行时可 `user_config_path:=/absolute/xxx.json` 覆盖。
- 结构：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `lidar_summary_info.lidar_type` | int | MID360 固定 `8` |
| `MID360.lidar_net_info.*_port` | int | 雷达侧端口：命令 56100、推送 56200、点云 56300、IMU 56400、日志 56500 |
| `MID360.host_net_info.*_ip` | string | 上位机网卡 IP，必须与现场一致 |
| `MID360.host_net_info.*_port` | int | 上位机侧端口，默认雷达端口 +1 |
| `lidar_configs[].ip` | string | 雷达 IP |
| `lidar_configs[].pcl_data_type` | int | 点云数据类型，当前 `1` |
| `lidar_configs[].pattern_mode` | int | 扫描模式，当前 `0` |
| `lidar_configs[].extrinsic_parameter` | object | 安装外参：`roll/pitch/yaw`（rad）、`x/y/z`（m） |

## 4. 算法参数（YAML）

### 4.1 FAST-LIO / FAST-LIO_LOCALIZATION

两块包使用同一套结构，文件在各自 `config/<雷达>.yaml`（MID360 用 `mid360.yaml`）：

| 分组 | 键 | 说明 |
| --- | --- | --- |
| `common` | `lid_topic`、`imu_topic` | 默认 `/livox/lidar`、`/livox/imu` |
| `common` | `time_sync_en`、`time_offset_lidar_to_imu` | 仅在无外部时间同步时开启 |
| `preprocess` | `lidar_type`、`scan_line`、`blind` | `1` Livox、`2` Velodyne、`3` Ouster；盲区单位 m |
| `mapping` | `acc_cov`、`gyr_cov`、`b_acc_cov`、`b_gyr_cov` | 噪声协方差 |
| `mapping` | `fov_degree`、`det_range` | 视场与探测距离 |
| `mapping` | `extrinsic_T`、`extrinsic_R`、`extrinsic_est_en` | 雷达到 IMU 外参；`est_en=false` 时使用固定值 |
| `publish` | `path_en`、`scan_publish_en`、`dense_publish_en` | 发布开关 |
| `pcd_save` | `pcd_save_en`、`interval` | `-1` 表示所有帧写入一个 PCD |

### 4.2 MoveBase3D

- 全局：`src/MoveBase3D/global_planner_3d/config/move_base_3d/move_base_global.yaml`，分 `map`（分辨率、膨胀系数、x/y 边界）与 `planning`（步长、底盘高度、到达阈值、邻域半径、平面/平坦度/坡度/稀疏度权重、收敛阈值、拟合半径）。
- 局部：同目录 `move_base_local.yaml`。
- 障碍：同目录 `move_base_global_obs.yaml`，配合 `/output_filter`。
- 地势：`rover_gpr/config/hyperparam.txt`。
- 这些参数带原地注释，改参数时同时改注释，避免语义漂移。

## 5. ROS 接口

### 5.1 主要话题（源码确认）

| 话题 | 类型 | 方向/说明 |
| --- | --- | --- |
| `/livox/lidar` | livox `CustomMsg` 或 `PointCloud2` | 驱动 → LIO |
| `/livox/imu` | `sensor_msgs/Imu` | 驱动 → LIO |
| `/cloud_registered` | `PointCloud2`（`camera_init`） | LIO 输出，过滤与可视化 |
| `/cloud_registered_body` | `PointCloud2`（`body`） | LIO 输出 |
| `/cloud_effected` | `PointCloud2` | LIO 输出 |
| `/Laser_map` | `PointCloud2` | LIO 输出 |
| `/Odometry` | `nav_msgs/Odometry`（`camera_init → body`） | LIO 里程计 |
| `/path` | `nav_msgs/Path` | LIO 轨迹 |
| `/map` | `PointCloud2`（`map`） | 先验地图，5 s 重发 |
| `/map_to_odom` | `nav_msgs/Odometry` | 定位模块的修正量 |
| `/localization` | `nav_msgs/Odometry`（`map → body`） | 融合后的全局定位 |
| `/output_filter` | `PointCloud2` | 实时障碍输入 |
| `/cmd_vel` | `geometry_msgs/Twist` | 规划 → 底盘 |
| `/odom`、`/imu`、`/PowerVoltage` | 底盘 | 底盘里程计、IMU、电压 |
| `/cur_goal`、`/global_path`、`/cur_global_goal_status` | MoveBase3D | 目标、全局路径、状态 |

### 5.2 消息定义

| 包 | 文件 | 关键字段 |
| --- | --- | --- |
| `dl_msgs` | `msg/Velocities.msg` | `linear_x`、`linear_y`、`angular_z`（float32） |
| `dl_msgs` | `msg/Imu.msg` | 三轴加速度、角速度、磁力计 |
| `dl_msgs` | 其余 msg/srv | 电池、继电器、舵机、超声等底盘外设 |
| `rover_msgs` | `roverGoalStatus`、`roverBmsStatus`、`roverRCStatus` | 导航与底盘状态 |
| `livox_ros_driver2` | `CustomMsg`、`CustomPoint` | 雷达原始点云 |
| `FAST_LIO*` | `Pose6D.msg` | 6 自由度位姿 |

### 5.3 参数透传

`tools/run.sh` 只做参数校验与透传，常用键：`mode`、`map`、`map_2d`、`publish_2d_map`、`rviz`、`save_map`、`user_config_path`、`usart_port_name`、`serial_baud_rate`。新增参数直接传给 `system.launch`，不要在脚本里硬编码设备路径。

## 6. 串口协议（固件 ↔ 上位机）

- 物理层：`/dev/ttyUSB0`，115200，8N1；可用 `usart_port_name`、`serial_baud_rate` 覆盖。
- 命令帧固定 11 字节，遥测帧固定 24 字节；帧头 `0x7B`，帧尾 `0x7D`，校验为前 N 字节异或。
- 命令帧：`x/y/z_cmd` 为 `int16_be`，发送前乘 1000。
- 遥测帧：速度、电压与诊断字段；`PROTOCOL_25` 必须为 `0`，否则变成 25 字节，驱动按 24 字节解析会错位。
- 当前 `TR800_MOTOR_TELEMETRY_DEBUG=1`，遥测第 8–19 字节是编码器/PWM/使能/状态码，不是 IMU；`tr800_serial_analyzer.py` 的字段名仍按 IMU 注释，读数据时以固件文档为准。
- 协议改动必须同步：固件 `HARDWARE/usartx.h` ↔ `src/dlrobot_robot/include/dlrobot_robot.h`（以及 upgrade 分支的 ROS 2 驱动）。

完整字节表见 `firmware/tr800_f407/docs/TR800_通信协议.md`。

## 7. 固件构建产物

- 产物：`dlrobot.elf`、`dlrobot.hex`、`dlrobot.bin`，位于 `build/tr800-st-preset-win/`。
- 构建入口：`CMakePresets.json` 的 `tr800-st`，工具链 `cmake/toolchains/tr800_stm32f407.cmake`。
- 烧录：DAPLink + OpenOCD（`openocd/tr800_daplink.cfg`、`scripts/windows/*.bat`）。
- 产物与 `build/`、`OBJ/` 均被 `.gitignore` 排除，不入库。

## 8. 依赖清单

- 版本固定用 `third_party/common.repos`，格式为 `repositories: <name>: {type: git, url, version}`，`version` 写完整 commit。
- 补丁为 unified diff，放在 `third_party/patches/<组件>/`，按文件名顺序 `git apply`。
- 安装顺序：`vcs import` → 应用补丁 → 按组件自己的 CMake 安装 → 构建 ROS 工作区。

## 9. 仓库与命名规范

- 分支：`main` 是 ROS 1 交付线，`upgrade/ros2` 是升级线，固件目录两线共用。
- ROS 包保持原包名与原目录层级；升级线按 `drivers/perception/localization/navigation/control/bringup` 划分。
- 文档中文；代码、注释、提交信息用正常散文；提交信息使用 `类型: 摘要`（如 `docs:`、`fix:`、`build:`）。
- 生成物（`build/`、`devel/`、`Log/`、`PCD/`、`*.pcd`、`*.bag`）不入库；地图与录包作为运行数据单独交付。
- `tools/` 只放薄封装，不复制算法逻辑。

## 10. 改动前的核对清单

1. 地图是否存在、是否绝对路径、坐标系是否与现场一致。
2. 雷达 JSON 的上位机 IP 是否为本机网卡地址。
3. FAST-LIO 外参是否需要重标定；`extrinsic_est_en` 是否应为 false。
4. 串口设备名、权限（`dialout` 组）、波特率是否与固件一致。
5. 协议字段是否同时改了固件与两个 ROS 驱动。
6. 新增话题或参数是否同步更新 `docs/modes.md` 与本文件。
