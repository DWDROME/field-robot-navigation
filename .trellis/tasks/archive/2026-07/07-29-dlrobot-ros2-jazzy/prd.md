# 迁移 dlrobot_robot 到 ROS 2 Jazzy

## Goal

将现有 `src/dlrobot_robot` 的底盘串口节点迁移为可在 Ubuntu 24.04、ROS 2 Jazzy 下构建和运行的 `ament_cmake` 包，为后续接入 CMU 官方 Jazzy 全栈提供底盘接口。

迁移只改变 ROS 工程形态和代码组织，不创新底盘算法，不改变下位机串口协议。

## Background

- 当前仓库仍以 ROS 1 Noetic 为主，`src/dlrobot_robot` 是 catkin 包。
- 当前节点订阅相对 Topic `cmd_vel`，消息类型为 `geometry_msgs/Twist`，见 `src/dlrobot_robot/src/dlrobot_robot.cpp:482-488`。
- 当前节点发布相对 Topic `odom`、`imu`、`PowerVoltage`，见 `src/dlrobot_robot/src/dlrobot_robot.cpp:482-484`。
- 当前启动参数为：
  - `usart_port_name=/dev/ttyACM0`
  - `serial_baud_rate=115200`
  - `odom_frame_id=odom_combined`
  - `robot_frame_id=base_footprint`
  - `gyro_frame_id=imu_link`
  见 `src/dlrobot_robot/launch/base_serial.launch:3-9`。
- 当前发送帧为 11 字节，接收帧为 24 字节，帧头、帧尾和异或校验规则见 `src/dlrobot_robot/include/dlrobot_robot.h:28-47` 与 `src/dlrobot_robot/src/dlrobot_robot.cpp:47-85,327-418`。
- 当前串口打开失败后仅记录错误并继续运行，见 `src/dlrobot_robot/src/dlrobot_robot.cpp:492-508`。ROS 2 版本不得保留这种隐藏失败状态。
- Smart Search 已确认可用，当前 Grok 搜索模型可访问 ROS 2 和 CMU 官方资料。实施前仍需 fetch 并核对具体页面及 CMU Jazzy 源码。

## Requirements

### R1. 独立工作区

- 原 ROS 1 包从 `src/dlrobot_robot` 迁移到 `other/历史归档/ros1/dlrobot_robot`。
- ROS 2 包直接重建在原路径 `src/dlrobot_robot`，不增加 `ros2_ws/` 目录层级。
- 新旧包的 ROS package name 均保持 `dlrobot_robot`，但不得位于同一构建工作区。

### R2. 官方迁移基线

- 以 ROS 2 Jazzy 官方迁移文档为工程依据。
- 使用 `ament_cmake`、`rclcpp`、ROS 2 Python launch 和 `colcon`。
- 核对 CMU `autonomy_stack_mecanum_wheel_platform` 的 Jazzy 分支，确认底盘命令 Topic、消息类型、串口设备和启动方式。

### R3. 保持底盘协议

- 保持 11 字节速度命令帧和 24 字节状态帧的字段顺序、比例、帧头、帧尾及异或校验。
- 保持速度、IMU 和电压的单位换算。
- 保持里程计积分、IMU 姿态计算和静止/运动两组协方差的现有行为；只修复迁移所必需的明显实现缺陷。

### R4. 保持 ROS 接口

- 默认保留 `cmd_vel`、`odom`、`imu`、`PowerVoltage` 的相对 Topic 名称。
- 为直接接入 CMU 官方 Jazzy 全栈，`cmd_vel` 使用 `geometry_msgs/msg/TwistStamped`；节点读取其中的 `twist` 字段。
- 保留输出消息类型、参数名和 frame 默认值。
- 串口节点是底盘串口的唯一所有者；接入本包时必须关闭 CMU `pathFollower` 自带的电机串口输出，避免两个进程同时写同一设备。
- 串口依赖使用 CMU 官方全栈同样采用的 ROS 2 `serial` 包，不在本包中复制第二套串口实现。

### R5. 直接、简洁、可诊断

- 串口协议解析与 ROS 节点职责分离，协议部分可脱离 ROS 做单元测试。
- 使用固定大小缓冲区、浅层控制流、RAII 和显式错误。
- 不使用全局可变状态、静默 fallback、假重连或吞异常。
- 串口无法打开或配置失败时节点必须显式失败。
- 不添加用户未要求的自动重连、速度看门狗、TF 广播、滤波器替换或控制算法。

### R6. Spec

- 在 `.trellis/spec/backend/` 增加 ROS 2 机器人 C++ 代码风格规范。
- 规范必须覆盖接口兼容、固定缓冲区、串口错误处理、参数、回调结构和最小测试要求，并加入 backend spec 索引。

### R7. 文档

- 在 `src/dlrobot_robot/README.md` 提供简洁说明，记录包用途、构建、启动、参数、Topic 和当前硬件未验证边界。
- 不编写验收表式长文档，不扩写算法研究内容。

## Acceptance Criteria

- [ ] `src/dlrobot_robot/package.xml` 为 ROS 2 format 3 包，并声明实际使用的依赖。
- [ ] `CMakeLists.txt` 使用 `ament_cmake`，可以由 `colcon build --packages-select dlrobot_robot` 构建。
- [ ] ROS 2 节点订阅 `geometry_msgs/msg/TwistStamped` 的 `cmd_vel`，发布 `nav_msgs/msg/Odometry`、`sensor_msgs/msg/Imu` 和 `std_msgs/msg/Float32`。
- [ ] 参数名和默认 frame 与 ROS 1 版本一致，默认串口启动值为 `/dev/ttyACM0`、115200。
- [ ] README 明确同一串口只能由一个节点控制，并说明接入 CMU 全栈时应关闭 `pathFollower` 自带串口输出。
- [ ] 发送帧与接收帧协议有不依赖 ROS 的测试，至少覆盖正常帧、错误校验、错误帧尾、噪声后重新同步和速度边界。
- [ ] 串口打开或配置失败时，节点启动失败并输出明确错误。
- [ ] 原 ROS 1 包完整迁移到 `other/历史归档/ros1/dlrobot_robot`，`src/dlrobot_robot` 不再保留旧 catkin 包。
- [ ] `src/dlrobot_robot/README.md` 简洁说明用途、接口、参数、构建和启动命令。
- [ ] 新增 ROS 2 代码风格 spec 已加入 `.trellis/spec/backend/index.md`。
- [ ] 已完成能在当前环境执行的最小构建、协议测试、launch 语法检查和 diff 检查；缺少 ROS 2 或真实底盘时明确列出未验证项。

## Out Of Scope

- FAST-LIO、FAR、FALCO、地形分析或其他导航算法修改。
- 改变 STM32 固件或串口协议。
- 自动重连、控制安全监督和新的速度限制策略。
- ROS 1 与 ROS 2 的运行时 bridge。
- 真实车辆参数标定和硬件实车验证。
