# Robot Navigation & SLAM

本分支 `main` 是原版 ROS 1 交付线：FAST_LIO 建图、FAST_LIO_LOCALIZATION 已有地图定位、MoveBase3D 三维导航，以及 Livox 和底盘驱动。

升级成果位于 `upgrade/ros2`：ROS 2 Jazzy 整车、ikd-Tree/OctVox 与回环扩展、FAR、CMU/MPPI、Gazebo Harmonic/PX4，以及独立 Noetic FAST-LIVO2。使用 `git switch upgrade/ros2` 切换；两条分支保留各自完整源码与构建入口。

## 模式

| 模式 | 内容 | 环境 |
| --- | --- | --- |
| mapping | 雷达、FAST_LIO、地图保存 | hardware |
| localization | 已有 PCD 地图定位与 TF | hardware |
| navigation | 定位、MoveBase3D、点云过滤、底盘 | hardware |

仿真环境在升级分支，入口和当前支持范围见[仿真说明](docs/simulation.md)。各版本中的算法参数与运行模式分别配置。

## 构建与启动

准备 Noetic、Livox-SDK2 及包依赖，详见[构建说明](docs/build.md)。

```bash
bash tools/build-ros1.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
bash tools/run.sh mapping hardware
bash tools/run.sh localization hardware map:=/absolute/map.pcd
bash tools/run.sh navigation hardware map:=/absolute/map.pcd
```

地图是运行输入，按实际位置提供 PCD 文件；源码不附带原有大型地图和录包。雷达 JSON、串口设备、RViz 与二维辅助地图参数见[模式说明](docs/modes.md)。旧 `start_all.sh` 保留为 navigation/hardware 的入口。

## 目录

```text
src/
  FAST_LIO/                 建图
  FAST_LIO_LOCALIZATION/    已有地图定位
  MoveBase3D/               规划、局部控制与整车启动
  livox_ros_driver2/        雷达驱动
  dlrobot_robot/            当前底盘串口入口
  dlrobot/、dl_msgs/        原有底盘包与消息
  livox_to_pointcloud2/     点云转换
  pcd2pgm_package/          PCD 转二维地图
firmware/tr800_f407/        下位机源码、构建与烧录
tools/                     构建与模式入口
third_party/               外部依赖来源
docker/                    版本对应的环境说明
docs/                      架构与操作说明
```

[系统架构](docs/architecture.md) 说明模块关系；[MoveBase3D](src/MoveBase3D/README.md) 保留原始框架图与导航参数。[TR800 固件](firmware/tr800_f407/README.md) 独立编译、烧录，ROS 构建只处理 src。

## 来源与许可

原版包的源码、版权头、LICENSE 和 README 使用声明保留。[第三方说明](third_party/README.md) 汇总来源；使用和分发遵循各包原始条款。
