# dlrobot_robot ROS 2 Jazzy 迁移设计

## 1. 边界

本任务只迁移底盘串口包。FAST-LIO、FAR、local planner、地形分析和 STM32 固件均不修改。

目录固定为：

```text
other/历史归档/ros1/dlrobot_robot/  # 原 ROS 1 包完整归档
src/dlrobot_robot/                  # 新 ROS 2 Jazzy 包
```

不创建 `ros2_ws/`，不保留同路径双实现。

## 2. ROS 2 包结构

```text
src/dlrobot_robot/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/dlrobot_robot/
│   ├── orientation_filter.hpp
│   └── protocol.hpp
├── launch/base_serial.launch.py
├── src/
│   ├── dlrobot_robot_node.cpp
│   ├── orientation_filter.cpp
│   └── protocol.cpp
└── test/test_protocol.cpp
```

使用 C++17、`ament_cmake`、`rclcpp`、Python launch 和 `colcon`。

## 3. 数据流

```text
CMU pathFollower
  -> cmd_vel (geometry_msgs/msg/TwistStamped)
  -> dlrobot_robot_node
  -> 11 字节速度命令
  -> STM32

STM32
  -> 24 字节状态帧
  -> dlrobot_robot_node
  -> odom / imu / PowerVoltage
```

`cmd_vel` 使用相对 Topic 名，默认在根命名空间下解析为 `/cmd_vel`，同时保留 remap 能力。

## 4. CMU 兼容边界

CMU Jazzy 分支 `71cafe32a2ab` 的 `pathFollower` 在 `/cmd_vel` 发布
`geometry_msgs/msg/TwistStamped`，因此本包直接订阅该类型，不额外增加 bridge。

CMU `pathFollower` 同时包含直接写电机串口的能力。本包接管底盘后，同一串口只允许本包持有；CMU 的串口输出必须关闭或不配置。这个约束写入 README，不在本包中添加自动仲裁。

## 5. 串口和协议

- 使用 ROS 2 `serial` 包，保持与 CMU 官方全栈一致。
- 发送帧固定 11 字节，接收帧固定 24 字节。
- 帧头、帧尾、字段位置、缩放和 XOR 校验保持不变。
- 协议编码、解码和流式重同步放入不依赖 ROS 的模块。
- 命令缩放到 `int16_t` 前做边界限制，避免数值溢出。
- 串口打开或配置失败时抛出错误并终止节点，不静默运行。
- 不实现自动重连或隐藏 fallback。

## 6. 状态发布

- `odom`: `nav_msgs/msg/Odometry`
- `imu`: `sensor_msgs/msg/Imu`
- `PowerVoltage`: `std_msgs/msg/Float32`

保留原包的 frame 参数、单位换算、姿态估计和协方差行为。里程计仅在收到有效状态帧后更新。

## 7. 测试

协议测试不启动 ROS，覆盖：

- 正常速度帧；
- 超范围速度限制；
- 正常状态帧；
- 校验错误；
- 帧尾错误；
- 噪声后的重新同步。

包级验证使用 Jazzy 环境下的 `colcon build`、`colcon test` 和 launch Python 语法检查。无真实底盘时不声称完成硬件验证。

## 8. 许可证与回滚

原包 `package.xml` 的许可证为 `TODO`。迁移不虚构许可证，继续保留并在 README 标记为待确认事项。

如需回滚，完整 ROS 1 包仍保存在 `other/历史归档/ros1/dlrobot_robot`，但本任务不自动恢复或维护双版本同步。
