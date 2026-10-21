# 系统架构

`main` 是 ROS 1 原版交付线，保留原有包布局。`upgrade/ros2` 保存 ROS 2 整车、地图与回环、FAR、CMU/MPPI、Gazebo 和独立 Noetic FAST-LIVO2。

## 原版数据流

Livox MID360 提供点云与 IMU。FAST_LIO 负责建图和连续里程计；FAST_LIO_LOCALIZATION 将里程计与已有 PCD 地图的全局定位结合，输出定位与 TF。MoveBase3D 使用地图、实时点云、目标和 TF，完成全局规划、GPR 地势分析及局部控制。点云过滤包提供实时障碍输入，dlrobot_robot 将速度命令转换为底盘串口帧。

| 职责 | 目录或 ROS 包 |
| --- | --- |
| 建图 | src/FAST_LIO，包名 fast_lio |
| 已有图定位 | src/FAST_LIO_LOCALIZATION，包名 fast_lio_localization |
| 导航 | src/MoveBase3D 内的 global_planner_3d、rover_gpr、local_planner_3d |
| 整车启动 | src/MoveBase3D/rover_launch/launch/system.launch |
| 雷达和底盘 | livox_ros_driver2、dlrobot_robot |
| 下位机 | firmware/tr800_f407 |

## 模式与环境

`mapping` 只装配雷达和建图；`localization` 装配雷达与已有地图定位；`navigation` 在定位基础上加入底盘、MoveBase3D 和点云过滤。入口为 `tools/run.sh`，各模式使用同一份算法源码。

main 当前使用 hardware 环境。simulation 环境位于 upgrade/ros2，其 Gazebo Harmonic 接入 ROS 2；ROS 1 仿真需要另外适配。ROS 1/ROS 2 包分别构建，F407 固件使用自己的交叉编译工具链。

模式、地图和设备参数见[操作说明](modes.md)，具体导航节点图与参数见 [MoveBase3D](../src/MoveBase3D/README.md)。
