# Field Robot Navigation

ROS 1 Noetic 下的激光建图、点云地图定位与户外三维导航工程，面向 Livox Mid-360 和 TR800 移动底盘。

`main` 保留现有实车软件链：FAST-LIO2 建图、FAST-LIO-LOCALIZATION 重定位、MoveBase3D 规划，以及串口底盘控制。ROS 2 Jazzy、Gazebo/PX4 仿真和扩展 SLAM 代码在 [`upgrade/ros2`](https://github.com/DWDROME/field-robot-navigation/tree/upgrade/ros2)。

## 系统怎么工作

```text
Mid-360 (LiDAR + IMU)
         │
         ├─ FAST-LIO2 ────────────────────────> PCD 地图
         │
         └─ FAST-LIO-LOCALIZATION + 既有 PCD ─> 定位 / TF
                                                   │
                      实时点云 ───────────────────┤
                                                   ▼
                                          MoveBase3D
                                  地势分析 / 全局与局部规划
                                                   │
                                                /cmd_vel
                                                   ▼
                                   dlrobot_robot → TR800 F407
```

建图、定位和导航分别运行，避免在尚未准备地图和标定的情况下直接启动整车控制。导航使用 MoveBase3D 的三维地形规划；底盘里程计未参与 FAST-LIO 的定位融合。具体 TF 关系见 [架构说明](docs/architecture.md)。

## 构建

目标环境：**Ubuntu 20.04 + ROS Noetic**。先安装 ROS、PCL、Eigen、OMPL、OctoMap 等依赖。定位、导航还需要包内所列的 Open3D、CasADi、`ros_numpy` 等组件；Livox-SDK2 需要单独安装，来源和补丁见 [third_party](third_party/README.md)。

在仓库根目录执行：

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash tools/build-ros1.sh
source devel/setup.bash
```

`build-ros1.sh` 使用 catkin，默认两个并行编译任务；F407 固件不参与 ROS 构建。依赖和环境说明见 [docs/build.md](docs/build.md)。

## 运行

```bash
# 1. 建图：正常 Ctrl+C 退出后保存 scans.pcd
bash tools/run.sh mapping hardware

# 2. 在已建点云地图中定位
bash tools/run.sh localization hardware map:=/absolute/map.pcd

# 3. 启动定位、规划和底盘控制
bash tools/run.sh navigation hardware map:=/absolute/map.pcd
```

定位和导航都必须传入已有的 PCD 文件。建图默认保存到 `src/FAST_LIO/PCD/scans.pcd`；正式运行前还需要按设备修改 Mid-360 网络地址、安装外参、底盘串口和导航参数。

`tools/run.sh` 可以透传 `roslaunch` 参数，例如 `rviz:=false`、`usart_port_name:=/dev/serial/by-id/...`。初始定位、二维辅助地图和其他参数见 [运行说明](docs/modes.md)。

## 代码在哪里

| 目录 | 内容 |
| --- | --- |
| [`src/FAST_LIO`](src/FAST_LIO) | FAST-LIO2 建图 |
| [`src/FAST_LIO_LOCALIZATION`](src/FAST_LIO_LOCALIZATION) | 基于 PCD 的全局定位 |
| [`src/MoveBase3D`](src/MoveBase3D) | 地形、全局规划与局部控制 |
| [`src/livox_ros_driver2`](src/livox_ros_driver2) | Mid-360 驱动 |
| [`src/dlrobot_robot`](src/dlrobot_robot) | ROS 与 TR800 底盘串口通信 |
| [`firmware/tr800_f407`](firmware/tr800_f407) | STM32F407 固件、构建与烧录 |
| [`tools`](tools) | 构建和运行脚本 |

## 当前状态

**截至 2026-10-21**，这一分支完成了源码、启动配置和文档的静态整理，**尚未在该整理版本上完成编译、节点启动或实车验证**。因此这里的命令是工程入口，不代表一键即用的交付保证。

当前最需要补的是 Noetic 全量构建、三种模式的 ROS 节点/TF 检查，以及 Mid-360 与底盘的现场标定。记录在 [状态与待办](docs/roadmap.md)。

## 更多资料

[系统架构](docs/architecture.md) · [构建依赖](docs/build.md) · [运行模式](docs/modes.md) · [地图、话题与串口协议](docs/data-formats.md) · [F407 固件](firmware/tr800_f407/README.md)

工程集成多个独立开源项目，保留其原始版权、许可证和使用约束；尤其请留意 MoveBase3D 上游 README 中的科研用途声明。外部依赖见 [third_party/README.md](third_party/README.md)。
