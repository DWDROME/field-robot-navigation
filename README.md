# Robot Navigation & SLAM

面向温室与户外履带机器人的定位、建图与导航工程。仓库用三个交付边界保存不同形态的成果，先确认自己在哪条线上，再读对应文档。

| 边界 | 内容 | 适合谁 |
| --- | --- | --- |
| `main`（本分支） | ROS 1 Noetic 原版交付线：FAST-LIO 建图、FAST-LIO_LOCALIZATION 已有地图定位、MoveBase3D 三维导航、Livox 与底盘驱动，共 18 个 ROS 包 | 要在实车上跑 ROS 1 链路的人 |
| `upgrade/ros2` | ROS 2 Jazzy 整车、ikd-Tree/OctVox 与回环、FAR、CMU/MPPI、Gazebo Harmonic/PX4，以及独立 Noetic FAST-LIVO2 | 做算法升级或仿真的人 |
| `firmware/tr800_f407`（两条分支同一份） | STM32F407 + FreeRTOS 下位机，独立 CMake/GCC/Ninja 构建与 OpenOCD 烧录 | 改底盘固件的人 |

三种运行模式统一由 `tools/run.sh` 启动：`mapping` 建图、`localization` 已有地图定位、`navigation` 定位加规划与底盘。当前进度、待办与风险见 [docs/roadmap.md](docs/roadmap.md)。

## 第一次接手，按这个顺序读

1. 本文件：先弄清三条线各自负责什么。
2. [docs/architecture.md](docs/architecture.md)：数据从雷达到底盘怎么流，每个包管哪一段。
3. [docs/build.md](docs/build.md)：把 Noetic 环境装起来。
4. [docs/modes.md](docs/modes.md)：三个模式怎么启动、参数怎么传。
5. 要改哪个模块，再读该包的 README；下位机读 [firmware/tr800_f407/README.md](firmware/tr800_f407/README.md)。

## 快速开始

### 1. 环境

- Ubuntu 20.04 + ROS Noetic。`ros-noetic-desktop-full` 能覆盖大部分依赖。
- 系统库：PCL、Eigen、OpenMP、OMPL、OctoMap、串口库；定位与规划包还需要 CasADi、Open3D、`ros_numpy`。
- Livox-SDK2 独立安装，版本与两个 `<cstdint>` 补丁见 [third_party/README.md](third_party/README.md)。ROS 驱动源码在 `src/livox_ros_driver2`，不随 SDK 一起编译。

### 2. 构建

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash tools/build-ros1.sh          # catkin_make -DROS_EDITION=ROS1；BUILD_JOBS 默认 2
source devel/setup.bash
```

`build-ros1.sh` 只构建 `src/`；`firmware/` 不参与 catkin 构建，也不生成固件。

### 3. 运行

```bash
# 建图：只启动雷达与 FAST-LIO；Ctrl+C 正常退出后保存到 src/FAST_LIO/PCD/scans.pcd
bash tools/run.sh mapping hardware

# 定位：必须提供已有 PCD 地图
bash tools/run.sh localization hardware map:=/absolute/map.pcd

# 导航：定位 + 底盘 + MoveBase3D + 实时点云过滤
bash tools/run.sh navigation hardware map:=/absolute/map.pcd
```

常用参数直接追加到命令末尾：`rviz:=false`、`save_map:=false`、`user_config_path:=/absolute/MID360_config.json`、`usart_port_name:=/dev/serial/by-id/…`、`serial_baud_rate:=115200`、`publish_2d_map:=true map_2d:=/absolute/map.yaml`。初始位姿用 RViz 的初始位姿工具给出，具体操作见 [docs/modes.md](docs/modes.md)。

旧入口 `bash start_all.sh map:=…` 等价于 `navigation hardware`，保留兼容。

### 4. 固件（与 ROS 构建无关）

按 [firmware/tr800_f407/README.md](firmware/tr800_f407/README.md) 使用 CMake preset 生成 `dlrobot.elf/hex/bin`，再用 DAPLink + OpenOCD 烧录。烧录后用 `firmware/tr800_f407/tools/tr800_serial_analyzer.py --port COM4 --diag-control` 核对当前控制诊断分页（设备名按现场修改）；命令帧为 11 字节，遥测帧为 24 字节。

底盘驱动只发布轮速积分里程计 `/odom` 与电压 `/PowerVoltage`，不发布串口 IMU。当前遥测 `8..19` 字节用于控制诊断；FAST-LIO 使用 `/livox/imu`。实际 TF 链及里程计消息的区别见 [docs/data-formats.md](docs/data-formats.md)。

## 仓库结构

```text
src/
  FAST_LIO/                 建图（FAST-LIO2）
  FAST_LIO_LOCALIZATION/    已有 PCD 地图定位
  MoveBase3D/               全局规划、地势分析、局部控制、整车启动（8 个包）
  livox_ros_driver2/        MID360 驱动（ROS1/ROS2 双配置）
  livox_to_pointcloud2/     点云转 PointCloud2
  pcd2pgm_package/          PCD 转二维栅格地图
  dlrobot_robot/            当前底盘串口入口
  dlrobot/、dl_msgs/        旧底盘包与消息定义
docs/                       架构、构建、模式、路线图
tools/                      run.sh 与 build-ros1.sh
firmware/tr800_f407/        下位机源码、构建与烧录
third_party/                Livox-SDK2 版本与补丁
docker/                     说明：main 不使用 ROS 2 容器
```

## 文档地图

| 文档 | 内容 |
| --- | --- |
| [docs/architecture.md](docs/architecture.md) | 模块关系、TF 与职责表 |
| [docs/build.md](docs/build.md) | 依赖、构建、运行输入 |
| [docs/modes.md](docs/modes.md) | 三种模式的装配与参数 |
| [docs/data-formats.md](docs/data-formats.md) | 地图、配置、话题、TF 与串口协议的数据格式 |
| [docs/roadmap.md](docs/roadmap.md) | 当前状态、下一步、风险 |
| [docs/simulation.md](docs/simulation.md) | 仿真为什么在升级线 |
| [firmware/tr800_f407/README.md](firmware/tr800_f407/README.md) | 固件构建、烧录、协议 |
| `src/*/README.md` | 各上游包的原始说明与用法 |

## 当前状态

已完成：仓库与分支整理、两条线的源码/包名/许可保全、启动入口与模式装配核对、文档本地链接检查；具体记录见 [docs/roadmap.md](docs/roadmap.md)。

未完成：Noetic 环境下的完整编译、固件编译与烧录、Docker 镜像重建、仿真运行、实机验证。也就是说，“能编译、能上车”目前仍是待验证项，不应对外当作已完成的结论。

已知限制：

- 升级线 PCD 定位尚未接入；main 的 `simulation` 入口会明确拒绝并提示切换分支。
- 升级线通用 hardware profile 的纯建图仍受限。
- 固件 `PROTOCOL_25` 必须保持为 `0`，否则遥测帧变成 25 字节，驱动按 24 字节解析会错位。
- 编码器第 2 路闭环稳定性是已知风险，见固件文档。

## 常见问题

| 现象 | 原因与处理 |
| --- | --- |
| 提示 Provide the existing 3D map | `localization` / `navigation` 必须传 `map:=/abs/map.pcd` |
| `roslaunch: command not found` | 没有 source Noetic 或 `devel/setup.bash` |
| 找不到 livox 驱动或 SDK 头文件 | 未安装 Livox-SDK2，按 [third_party/README.md](third_party/README.md) 安装 |
| 串口打不开 | 设备名不对，或当前用户不在 `dialout` 组；确认波特率与固件一致 |
| 雷达没有点云 | `MID360_config.json` 中的 IP/端口与实际网络不符 |
| 地图没有保存 | 需要 Ctrl+C 正常退出，且 `src/FAST_LIO/PCD` 可写 |
| 想跑仿真 | main 没有 ROS 1 仿真适配，切换到 `upgrade/ros2` |

## 交接说明

- 两条分支各自完整，`git switch` 就是切换版本；`firmware/` 两线实现一致，只有 README 不同。
- 地图与录包属于运行数据，不入库；用 `map:=` 指向实际文件。
- 本机的 `.worktrees/`、`.cache/`、`.tmp/` 是审计与工作材料，不属于分支内容；删除 `.worktrees` 会丢掉归档 worktree，动手前先确认。
- 串口协议改动必须同时改固件 `HARDWARE/usartx.h`、main 驱动 `src/dlrobot_robot/include/dlrobot_robot.h` 与升级线驱动，否则车不动。
- 仓库没有 CI；每次改动至少跑一次 `bash tools/run.sh --help`、相关包构建与 launch 语法检查。

## 来源与许可

原版包名、版权头、LICENSE 与 README 使用声明全部保留；第三方来源见 [third_party/README.md](third_party/README.md)。MoveBase3D 原始 README 注明“仅供科研测试，请勿商用”，其余包遵循各自原始条款。
