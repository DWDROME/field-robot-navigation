# Field Robot Navigation · ROS 2

一个面向户外移动机器人的 ROS 2 Jazzy 导航实验工程。重点是把 **LiDAR 惯性里程计、三维地形、路径规划和底盘控制** 接成同一条可检查的链路，并在 Gazebo 中复现农业场景。

本分支是 `upgrade/ros2`。原始 ROS 1 Noetic 实车代码在 [`main`](https://github.com/DWDROME/field-robot-navigation/tree/main)。两条线使用相同的 TR800 F407 固件，但构建和启动方式不同。

## 系统结构

```text
Livox Mid-360
  ├─ LiDAR / IMU → FAST-LIO → 里程计 ─────────────────────────┐
  │                       └─ ikd-Tree / OctVox 地图         │
  │                          Scan Context + GICP + iSAM2    │
  └─ 注册点云 → 点云规范化 → 地形分析 → FAR 全局路径          │
                                            │               │
                               ┌────────────┴───────┐       │
                               ▼                    ▼       │
                        CMU local planner       Nav2 MPPI    │
                        + path follower        (可选)       │
                               └─────────┬──────────┘       │
                                         ▼                  │
                                   速度适配 / 命令门控 ◀──────┘
                                         ▼
                                dlrobot_robot → TR800
```

默认控制链是 **FAR + CMU**；部署文件设置 `controllers: [mppi]` 可切换至 **FAR + Nav2 MPPI**，二者不同时持有速度命令。CMU 当前只按位置判断航点到达；需要终点朝向时使用 MPPI。控制门控会在数据过期、任务取消或控制来源不合法时请求停车。

地图后端支持 ikd-Tree 和 OctVox，回环使用 Scan Context 候选检索、GICP 验证和 GTSAM/iSAM2 优化。校正作用于 `map → odom`，局部里程计保持连续。算法接口与设计边界见 [架构文档](docs/architecture.md)。

## 从哪里开始

需要 **ROS 2 Jazzy、amd64 和 Docker BuildKit / Compose 2.17+**。默认生产镜像构建入口：

```bash
bash tools/build-ros2.sh
```

实车导航从部署配置读取 Mid-360 网络、外参、底盘设备、速度限制与急停接口：

```bash
bash tools/run.sh navigation hardware deployment:=/absolute/deployment.yaml
```

这条命令要求已有可运行的 Jazzy 环境与经过配置的硬件，不会自动完成标定。部署模板在 [`deployment-hardware.template.yaml`](src/navigation/greenhouse_mppi_navigation/config/deployment-hardware.template.yaml)；容器挂载和实际启动方式见 [docker/README.md](docker/README.md)。

仿真入口：

```bash
# 先按 docs/simulation.md 构建仿真镜像与运行环境
bash tools/run.sh navigation simulation world:=maize_field robot:=husky_a200
```

`world` 可选 `maize_field`、`orchard`、`pipeline`，`robot` 可选 `jackal_j100`、`husky_a200`、`x500`。运行前须完成 Gazebo、机器人模型、PX4（使用 X500 时）和 Compose 环境准备；完整步骤以 [仿真说明](docs/simulation.md) 为准。

本分支还维护两项独立算法工作区，**不要与 Jazzy 整车构建混在一起**：

```bash
bash tools/build-ros1.sh .tmp/ros1-slam
bash tools/build-fastlivo2.sh .tmp/fastlivo-build
```

前者是 FAST-LIO 地图/回环的 Noetic 工作区，后者是单独维护的 FAST-LIVO2（包含图像存储和地图内存管理）。依赖与运行参数见 [构建说明](docs/build.md)。

## 仿真做到哪一步

包含三个场景：玉米田、果园、丘陵；地面平台为 Jackal J100、Husky A200，空中平台为 PX4 X500。X500 做航点巡检，**不具备三维避障或地形跟随**。

2026-10-07 的历史候选镜像测试记录了 9 种组合、每种 3 次独立重置：

| 平台 | 结果 | 主要问题 |
| --- | --- | --- |
| Jackal / Husky | **0 / 18 通过** | 近场未知区域、传感器与控制链时序 |
| X500 | **8 / 9 通过** | 丘陵场地一次起降区越界失败 |

这是**历史仿真结果**，并非当前源码重新构建后的验收：截至 2026-10-21，镜像尚未重建、九组合尚未复跑、实车尚未验证。地面任务仍未跑通，不能据此声称完成闭环导航。每次测试的指标、失败原因与复现命令见 [docs/simulation.md](docs/simulation.md)；当前问题与下一步见 [docs/roadmap.md](docs/roadmap.md)。

## 代码与文档

| 位置 | 内容 |
| --- | --- |
| [`src/localization`](src/localization) | FAST-LIO 地图、回环、OctVox、独立 FAST-LIVO2 |
| [`src/perception`](src/perception) | 点云规范化与地形分析 |
| [`src/navigation`](src/navigation) | FAR、CMU、MPPI 和航点任务 |
| [`src/control`](src/control) | 控制命令门控 |
| [`src/bringup`](src/bringup) | 整车启动、部署校验、仿真 |
| [`src/drivers`](src/drivers) | Livox 和底盘驱动 |
| [`firmware/tr800_f407`](firmware/tr800_f407) | STM32F407 固件 |

[导航与任务接口](src/navigation/greenhouse_mppi_navigation/README.md) · [地图及回环](src/localization/fast_lio_super/README.md) · [FAST-LIVO2](src/localization/fast_livo/README.md) · [构建](docs/build.md) · [仿真](docs/simulation.md) · [话题与部署配置](docs/data-formats.md)

本库组合并修改了多项第三方工程。源码、版本锁和补丁见 [third_party/README.md](third_party/README.md)，原有许可证与署名保留；各组件仍遵循其各自的使用条件。
