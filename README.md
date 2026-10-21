# Robot Navigation & SLAM（upgrade/ros2）

本分支是 ROS 2 Jazzy 升级线：整车装配、ikd-Tree/OctVox 地图与回环、FAR 全局规划、CMU/MPPI 局部控制、Gazebo Harmonic/PX4 仿真，以及独立 Noetic FAST-LIVO2。原版 ROS 1 实车线与三模式在 `main`；`firmware/tr800_f407` 两条线共用。

> 状态：装配合同与文档已核对；镜像未重建、仿真矩阵未复跑、未实机。见 [docs/roadmap.md](docs/roadmap.md)。
> 本文只讲能力、构建入口和运行方式；依赖细节在 [docs/build.md](docs/build.md)，仿真在 [docs/simulation.md](docs/simulation.md)。

## 能力

| 主线 | 功能 | 入口 |
| --- | --- | --- |
| FAST-LIO2 SLAM | ikd-Tree / OctVox 后端、Scan Context 检索、GICP 验证、iSAM2 回环 | [定位建图](src/localization/fast_lio_super/README.md) |
| ROS 2 导航 | Mid-360、地形分析、FAR、默认 CMU、可选 Nav2 MPPI、命令门控与多航点 | [导航操作](src/navigation/greenhouse_mppi_navigation/README.md) |
| FAST-LIVO2 | 图像与地图预算、冷热存储、输入队列与退出生命周期 | [构建与配置](src/localization/fast_livo/README.md) |

| 控制模式 | 全局规划 | 局部控制 | 终点朝向 |
| --- | --- | --- | --- |
| 默认 CMU | FAR | CMU planner + follower | 只判位置到达，不保证 yaw |
| 可选 MPPI | FAR 完整图路径 | Nav2 MPPI | 按配置的 yaw 容差判定 |

部署配置里 `controllers: [cmu]` 或 `[mppi]` 二选一。CMU local Path 尚无任务身份，陈旧输入边界见[架构文档](docs/architecture.md#坐标与控制权)。

## 构建

| 目标 | 命令 |
| --- | --- |
| Jazzy 导航镜像 | `bash tools/build-ros2.sh` |
| Noetic SLAM | `bash tools/build-ros1.sh .tmp/ros1-slam` |
| FAST-LIVO2 | `bash tools/build-fastlivo2.sh .tmp/fastlivo-build` |

Jazzy 需要 Docker BuildKit、Compose 2.17+、amd64。Noetic 依赖可先固定来源：

```bash
python3 tools/prepare-dependencies.py ros1 .tmp/slam-deps gtsam livox_sdk2
```

三个入口的前置条件与注意事项见 [docs/build.md](docs/build.md)。构建与测试的临时目录统一放 `.tmp/`。

## 运行

```bash
# 实车整车
bash tools/run.sh navigation hardware deployment:=/absolute/deployment.yaml

# 仿真导航
bash tools/run.sh navigation simulation world:=maize_field robot:=husky_a200

# 纯建图仿真：只起建图与 TF，时钟和传感器由外部提供；启动 Gazebo 时加 navigation:=false
bash tools/run.sh mapping simulation
```

升级版 PCD 定位与通用 profile 的实车纯建图未接入，入口会拒绝并退出。整车入口为 `greenhouse_nav2_bringup/closed_loop.launch.py`，启动前需要传感器标定、底盘设备、运动限值与急停接口。

底盘串口只提供轮速里程计与电压，不发布底盘 IMU；遥测 `8..19` 字节是控制诊断，用 `--diag-control` 读取，导航的 `/sensors/imu/data` 来自 Livox。

## 仿真

```bash
bash tools/prepare-simulation.sh
BUILD_JOBS=1 docker compose -f docker/compose.sim.yaml build simulation-build
BUILD_JOBS=1 bash tools/build-px4.sh
BUILD_JOBS=1 docker compose -f docker/compose.sim.yaml build simulation
```

运行必须叠加 `docker/compose.sim.runtime.yaml`，WSL2 再加 `docker/compose.sim.wsl.yaml`。矩阵、场景、证据与已知失败见 [docs/simulation.md](docs/simulation.md)。

## 目录

```text
src/
  drivers/       传感器和底盘驱动
  perception/    点云规范化与地形分析
  localization/  地图、里程计与回环
  navigation/    路径规划和任务执行
  control/       速度命令门控
  bringup/       系统启动、配置校验与仿真入口
docker/          镜像构建、Compose 与运行配置
firmware/        底盘固件（STM32F407 + FreeRTOS）
tools/           构建与运行入口
third_party/     外部源码版本和补丁
docs/            架构、构建、仿真、数据格式、状态
```

## 文档

| 文档 | 内容 |
| --- | --- |
| [docs/architecture.md](docs/architecture.md) | 数据流、TF、控制权 |
| [docs/build.md](docs/build.md) | 三种构建入口 |
| [docs/simulation.md](docs/simulation.md) | 矩阵、场景、失败记录与证据 |
| [docs/data-formats.md](docs/data-formats.md) | 接口、QoS、profile、部署与仿真数据 |
| [docs/roadmap.md](docs/roadmap.md) | 状态、待办、风险 |
| [docker/README.md](docker/README.md) | 镜像与容器 |
| [third_party/README.md](third_party/README.md) | 来源、版本锁、补丁 |

## 交接

- 构建产物、镜像 tag、`.tmp/` 都不入库；`third_party/` 固定源码版本，重建前确认网络与代理。
- 仿真媒体与完整证据在本机归档，仓库只留结论、命令和必要数据。
- 串口协议改动要同步固件、本线 `src/drivers/dlrobot_robot` 与 main 的 ROS 1 驱动。
- 仓库没有 CI；改动后至少跑相关包构建、`colcon test` 与入口冒烟。

## 来源与许可

工程集成 FAST-LIO、Super-LIO/OctVox、FAST-LIVO2、Livox SDK/驱动、CMU 导航、FAR Planner、Nav2 与 GTSAM。[第三方登记](third_party/README.md) 汇总来源和版本锁。各包保留原始许可声明，使用与分发遵循对应条款。
