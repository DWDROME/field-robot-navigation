# Robot Navigation & SLAM

面向温室与户外履带机器人的定位、建图和自主导航工程。ROS 2 Jazzy 负责整车装配，ROS 1 Noetic 提供独立算法工作区。

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

## 来源与许可

工程集成 FAST-LIO、Super-LIO/OctVox、FAST-LIVO2、Livox SDK/驱动、CMU 导航、FAR Planner、Nav2 与 GTSAM。[第三方登记](third_party/README.md) 汇总来源和版本锁。各包及源文件保留原始许可声明，使用和分发遵循对应条款。
