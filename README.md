# 面向温室与户外机器人的定位建图与自主导航系统

面向温室与户外履带机器人的定位、建图和自主导航工程。ROS 2 Jazzy 负责整车装配，ROS 1 Noetic 提供独立算法工作区。

本项目以真实机器人部署为目标，包含传感器与底盘驱动、定位建图、路径规划、控制门控和整车启动配置。Gazebo 仿真用于验证集成链路并记录任务表现；仿真截图和测试结果不替代实机标定、现场调试与实机验收。

| 主线 | 功能 | 入口 |
| --- | --- | --- |
| FAST-LIO2 SLAM | ikd-Tree / OctVox 地图后端、Scan Context 检索、GICP 验证、iSAM2 回环校正 | [定位建图](src/localization/fast_lio_super/README.md) |
| ROS 2 导航 | Mid-360、地形分析、FAR、默认 CMU 局部规划与跟随、可选 Nav2 MPPI、命令门控与多航点 | [导航操作](src/navigation/greenhouse_mppi_navigation/README.md) |
| FAST-LIVO2 | 地图与图像预算、冷热图像存储、输入队列与退出生命周期 | [构建与配置](src/localization/fast_livo/README.md) |

## 目录

```text
src/
  drivers/       传感器和底盘驱动
  perception/    点云规范化与地形分析
  localization/  地图、里程计与回环
  navigation/    路径规划和任务执行
  control/       速度命令门控
  bringup/       系统启动和配置校验
docker/          构建环境、镜像和容器运行
tools/           构建入口
third_party/     外部源码版本和补丁
docs/            当前架构与构建说明
```

## 使用

从 [构建说明](docs/build.md) 选择 Jazzy 导航、Noetic SLAM 或 FAST-LIVO2。整车入口为 `greenhouse_nav2_bringup/closed_loop.launch.py`，启动前提供传感器标定、底盘设备、运动限值与急停接口。仿真配置需要外部时钟和传感器输入。

[系统架构](docs/architecture.md) 描述数据流、TF 和命令所有权；[容器说明](docker/README.md) 提供镜像构建与运行配置。

| 导航模式 | 全局规划 | 局部规划与控制 | 终点朝向 |
| --- | --- | --- | --- |
| 默认 CMU | FAR | CMU planner + follower | 仅判断位置到达，不保证终点 yaw |
| 可选 MPPI | FAR 完整图路径 | Nav2 MPPI | 按配置的 yaw 容差判断到达 |

通过部署配置 `controllers: [cmu]` 或 `[mppi]` 选择一种模式。CMU local Path 尚未携带显式任务身份，当前陈旧输入防护的边界见[系统架构](docs/architecture.md#坐标与控制权)。

## 仿真矩阵（三场地 × 三机器人）

Gazebo Harmonic 仿真矩阵在 [仿真说明](docs/simulation.md) 中给出准备、构建、启动、批量测试与截图流程。

| 场地 | 世界来源 | 场景特点 |
| --- | --- | --- |
| 玉米田 maize_field | [virtual_maize_field](https://github.com/FieldRobotEvent/virtual_maize_field)（ros2-gz） | 12 行直行+曲行作物、逐行缺株、地头、起伏与沟渠；构建期固定 seed 预生成 |
| 果园 orchard | clearpath_gz `worlds/orchard.sdf` | 成排树木、长通道、小坡 |
| 丘陵 pipeline | clearpath_gz `worlds/pipeline.sdf` | 丘陵高差、河流、桥梁、洞穴、管线、太阳能设施 |

| 机器人 | 类型 | 仿真范围 |
| --- | --- | --- |
| Clearpath Jackal J100 | 地面（差速） | 模拟传感器定位、地形分析 / FAR / MPPI / 命令门控与底盘接口 |
| Clearpath Husky A200 | 地面（差速） | 同链路；独立 footprint、传感器外参与运动限值 |
| PX4 X500（x500_depth） | 四旋翼（PX4 SITL v1.17.0） | 空中航点巡检与相机感知展示；未实现三维避障与地形跟随，不宣称支持 |

地面车与四旋翼的范围分别陈述：地面 `/cmd_vel/nav` 控制链只接地面车；X500 通过 PX4 Offboard 航点执行，与地面导航链相互独立。九组合（3 场地 × 3 机器人）自动测试的启动入口与结果格式见 [仿真说明](docs/simulation.md#九组合批量测试)。

### 2026-10-07 实测

| 场地 | Jackal J100 | Husky A200 | X500 |
| --- | --- | --- | --- |
| 玉米田 | fail × 3 | fail × 3 | pass × 3 |
| 果园 | fail × 3 | fail × 3 | pass × 3 |
| 丘陵 | fail × 3 | fail × 3 | fail × 1，pass × 2 |

九组合各三次独立重置；结果以实际任务状态计数，失败和阻塞保留完整记录。地面链接入真实模拟传感器定位、地形/FAR、MPPI、门控与物理底盘，目前任务因控制失败或定位数据过期而失败并停车，尚未验证成功导航或正确危险区拒绝。空中结果仅为空中航点巡检，不代表三维避障或地形跟随。

[完整相册（15 张）](docs/media/simulation/README.md) · [结果 JSON](docs/media/simulation/evidence/candidate-matrix/matrix_results.json) · [CSV](docs/media/simulation/evidence/candidate-matrix/matrix_results.csv) · [运行说明与命令](docs/simulation.md)

![玉米田与 Jackal J100](docs/media/simulation/maize_field_jackal_j100_r1_task.jpg)

![果园与 Husky A200](docs/media/simulation/orchard_husky_a200_r1_task.jpg)

![丘陵与 X500](docs/media/simulation/pipeline_x500_r1_task.jpg)

## 来源与许可

工程集成 FAST-LIO、Super-LIO/OctVox、FAST-LIVO2、Livox SDK/驱动、CMU 导航、FAR Planner、Nav2、GTSAM，以及仿真矩阵的 virtual_maize_field 生成器（GPL-3.0；玉米模型另为 CC BY-SA 4.0）、Clearpath 平台与世界资源、PX4-Autopilot（BSD-3-Clause）、Micro-XRCE-DDS-Agent（Apache-2.0）及 pymavlink。[第三方登记](third_party/README.md) 分别列明代码、模型、纹理的来源、署名和版本锁。各包及源文件保留原始许可声明。
