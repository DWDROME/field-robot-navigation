# Robot Navigation & SLAM（upgrade/ros2）

本分支保存 ROS 2 升级成果：Jazzy 整车装配、ikd-Tree/OctVox 地图与回环、FAR 全局规划、CMU/MPPI 局部控制、Gazebo Harmonic/PX4 仿真，以及独立 Noetic FAST-LIVO2。原版 ROS 1 交付线在 `main`，用 `git switch main` 切换；两条分支都保留同一份 `firmware/tr800_f407` 下位机源码。

当前进度、待办与风险见 [docs/roadmap.md](docs/roadmap.md)。

## 三条边界

| 边界 | 内容 | 适合谁 |
| --- | --- | --- |
| `main` | ROS 1 Noetic 原版交付线，18 个包，实车建图/定位/导航 | 要跑实车 ROS 1 链的人 |
| `upgrade/ros2`（本分支） | ROS 2 Jazzy 整车、仿真与算法升级 | 做算法、仿真或 ROS 2 集成的人 |
| `firmware/tr800_f407`（两线同一份） | STM32F407 + FreeRTOS 下位机 | 改底盘固件的人 |

## 第一次接手，按这个顺序读

1. 本文件：能力范围与入口。
2. [docs/architecture.md](docs/architecture.md)：数据流、TF 与控制权划分。
3. [docs/build.md](docs/build.md)：三种构建入口的依赖与命令。
4. 要仿真，读 [docs/simulation.md](docs/simulation.md)：镜像、场景、矩阵与已知失败。
5. 要实车 ROS 1 链路，切到 `main` 读它的 README。

## 能力范围

| 主线 | 功能 | 入口 |
| --- | --- | --- |
| FAST-LIO2 SLAM | ikd-Tree / OctVox 地图后端、Scan Context 检索、GICP 验证、iSAM2 回环校正 | [定位建图](src/localization/fast_lio_super/README.md) |
| ROS 2 导航 | Mid-360、地形分析、FAR、默认 CMU 局部规划与跟随、可选 Nav2 MPPI、命令门控与多航点 | [导航操作](src/navigation/greenhouse_mppi_navigation/README.md) |
| FAST-LIVO2 | 地图与图像预算、冷热图像存储、输入队列与退出生命周期 | [构建与配置](src/localization/fast_livo/README.md) |

| 导航模式 | 全局规划 | 局部规划与控制 | 终点朝向 |
| --- | --- | --- | --- |
| 默认 CMU | FAR | CMU planner + follower | 仅判断位置到达，不保证终点 yaw |
| 可选 MPPI | FAR 完整图路径 | Nav2 MPPI | 按配置的 yaw 容差判断到达 |

通过部署配置 `controllers: [cmu]` 或 `[mppi]` 选择一种模式。CMU local Path 尚未携带显式任务身份，陈旧输入防护的边界见[系统架构](docs/architecture.md#坐标与控制权)。

## 快速开始

### 1. 构建入口（按目标三选一）

| 目标 | 命令 | 前置条件 |
| --- | --- | --- |
| Jazzy 导航镜像 | `bash tools/build-ros2.sh` | Docker BuildKit、Compose 2.17+、amd64；首次需镜像/Git/软件包源 |
| Noetic SLAM（fast_lio_super 等） | `bash tools/build-ros1.sh .tmp/ros1-slam` | Noetic、GTSAM 4.2、Livox-SDK2；目标目录须不存在 |
| FAST-LIVO2 | `bash tools/build-fastlivo2.sh .tmp/fastlivo-build` | Noetic、GCC 9、OpenCV、PCL |

Noetic 依赖可先用脚本固定来源：

```bash
python3 tools/prepare-dependencies.py ros1 .tmp/slam-deps gtsam livox_sdk2
```

各入口的完整依赖、版本与注意事项以 [docs/build.md](docs/build.md) 为准。构建与测试的临时目录统一放 `.tmp/`。

### 2. 运行

统一入口为 `bash tools/run.sh <模式> <环境> [launch 参数...]`，模式为 `mapping`、`localization`、`navigation`，环境为 `hardware`、`simulation`。

```bash
# 实车整车
bash tools/run.sh navigation hardware deployment:=/absolute/deployment.yaml

# 仿真导航（已有世界）
bash tools/run.sh navigation simulation world:=maize_field robot:=husky_a200

# 纯建图仿真：只起建图与 TF，时钟和传感器由外部提供；启动 Gazebo 时加 navigation:=false
bash tools/run.sh mapping simulation
```

升级版 PCD 定位和通用 profile 的实车纯建图尚未接入，入口会明确提示并退出，不会切换成其他模式。原版三个实车模式在 `main`。

整车入口是 `greenhouse_nav2_bringup/closed_loop.launch.py`，启动前需要传感器标定、底盘设备、运动限值与急停接口。

### 3. 仿真

仿真矩阵、镜像构建、九组合批量测试与截图流程见 [docs/simulation.md](docs/simulation.md)。要点：

```bash
bash tools/prepare-simulation.sh
BUILD_JOBS=1 docker compose -f docker/compose.sim.yaml build simulation-build
BUILD_JOBS=1 bash tools/build-px4.sh
BUILD_JOBS=1 docker compose -f docker/compose.sim.yaml build simulation
```

运行必须叠加 `docker/compose.sim.runtime.yaml`；WSL2 再加 `docker/compose.sim.wsl.yaml`。

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
docs/            架构、构建、仿真与路线图
```

## 文档地图

| 文档 | 内容 |
| --- | --- |
| [docs/architecture.md](docs/architecture.md) | 数据流、TF、控制权与坐标约定 |
| [docs/build.md](docs/build.md) | Jazzy / Noetic / FAST-LIVO2 三种构建 |
| [docs/simulation.md](docs/simulation.md) | 仿真矩阵、场景、已知失败与证据 |
| [docs/data-formats.md](docs/data-formats.md) | 接口合同、QoS、profile、部署与仿真数据格式 |
| [docs/roadmap.md](docs/roadmap.md) | 当前状态、待办与风险 |
| [docker/README.md](docker/README.md) | 镜像与容器运行 |
| [third_party/README.md](third_party/README.md) | 来源、版本锁与补丁 |
| [导航说明](src/navigation/greenhouse_mppi_navigation/README.md) | 任务、航点与控制器操作 |

## 状态与限制

已经做过：升级源码的装配/launch 合同检查（24 项 pytest）、16 项入口分派检查、两线文档链接检查。仿真矩阵在 2026-10-07 有一轮完整记录：27 次独立重置中 8 次通过、19 次失败；18 次地面任务全部失败，空中玉米田/果园各 3 次通过、丘陵 2 次通过。该结果说明地面导航链路尚未达到可用，不能当作仿真通过。

未完成：本轮没有重建 production/SIM 镜像、没有复跑矩阵、没有实机验证。镜像 digest、源码 SHA 与失败日志见 [docs/simulation.md](docs/simulation.md)，复跑时按原文口径记录，不要删除失败记录或放宽阈值。

已知限制：

- 升级版 PCD 定位未接入，入口直接拒绝。
- 通用 hardware profile 的纯建图仍受标定配置限制。
- 地面任务受近场观测覆盖与数据新鲜度影响，出现零速安全停机；不要用“未知区置空闲”、缩小 footprint 或放宽新鲜度来掩盖。
- CMU local Path 没有显式任务身份，陈旧输入防护边界见架构文档。
- X500 只是航点巡检与感知展示，不做三维避障与地形跟随。

## 交接说明

- 构建产物、镜像 tag 与 `.tmp/` 工作区都不入库；`third_party/` 固定了源码版本，重建前先确认网络与代理。
- 仿真媒体与证据保存在本机归档，不进仓库；仓库只保留结论、命令和必要数据。
- 固件在两条分支是同一份实现，只有 README 不同；串口协议改动要同步固件、ROS 2 驱动 `src/drivers/dlrobot_robot` 与 main 的 ROS 1 驱动。
- 仓库没有 CI；改动后至少跑相关包构建、`colcon test` 与入口冒烟。

## 来源与许可

工程集成 FAST-LIO、Super-LIO/OctVox、FAST-LIVO2、Livox SDK/驱动、CMU 导航、FAR Planner、Nav2 与 GTSAM。[第三方登记](third_party/README.md) 汇总来源和版本锁。各包及源文件保留原始许可声明，使用和分发遵循对应条款。
