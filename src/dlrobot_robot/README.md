# dlrobot_robot

ROS 2 Jazzy 底盘串口节点，保持原 STM32 的 11 字节命令帧和 24 字节状态帧。

## 接口

- 订阅 `cmd_vel`：`geometry_msgs/msg/TwistStamped`
- 发布 `odom`：`nav_msgs/msg/Odometry`
- 发布 `imu`：`sensor_msgs/msg/Imu`
- 发布 `PowerVoltage`：`std_msgs/msg/Float32`

参数默认值：

```text
usart_port_name: /dev/ttyACM0
serial_baud_rate: 115200
odom_frame_id: odom_combined
robot_frame_id: base_footprint
gyro_frame_id: imu_link
```

## 构建与启动

本包依赖 CMU Jazzy 全栈使用的 ROS 2 `serial` 包。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select serial dlrobot_robot
source install/setup.bash
ros2 launch dlrobot_robot base_serial.launch.py
```

CMU `pathFollower` 默认发布 `/cmd_vel` 的 `TwistStamped`，可直接连接本节点。同一底盘串口只能由一个进程持有；使用本包时必须关闭 `pathFollower` 自带的电机串口输出。

原 ROS 1 包归档于 `other/历史归档/ros1/dlrobot_robot`。当前迁移未在真实底盘上验证，原包许可证字段仍为 `TODO`，需要后续确认来源许可证。
