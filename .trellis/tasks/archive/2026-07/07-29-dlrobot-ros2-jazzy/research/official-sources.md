# 官方依据

## ROS 2 Jazzy

- Package migration:
  `https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Packages.html`
- C++ package migration example:
  `https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/How-To-Guides/Migrating-from-ROS1/Migrating-CPP-Package-Example.rst`
- Parameter migration:
  `https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/How-To-Guides/Migrating-from-ROS1/Migrating-Parameters.rst`

采用结论：`ament_cmake`、`rclcpp`、Python launch、`colcon`、节点内声明参数、C++17 和显式编译警告。

## CMU Jazzy 全栈

- Repository:
  `https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform`
- 核对分支：`jazzy`
- 核对提交：`71cafe32a2abd4cba3f9d7c0fb1822456c5da3bc`
- 核对日期：2026-06-06

源码事实：

- `pathFollower` 在绝对 Topic `/cmd_vel` 发布 `geometry_msgs/msg/TwistStamped`。
- 官方工作区提供 ROS 2 `serial` 包。
- 默认电机串口为 `/dev/ttyACM0`，实际设备也可能是 `/dev/ttyACM1`。
- `pathFollower` 自带串口控制逻辑，因此接入 `dlrobot_robot` 时必须保证只有一个串口所有者。
