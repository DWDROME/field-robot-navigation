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

本包依赖 ROS 2 `serial` 包，固定来源见 `third_party/ros2.repos`。完整构建见 [构建说明](../../../docs/build.md)。已有 serial 工作区时先加载它，再构建本包：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/drivers/dlrobot_robot --packages-select dlrobot_robot
source install/setup.bash
ros2 launch dlrobot_robot base_serial.launch.py
```

整车装配由 `greenhouse_cmd_gate` 独占发布 `/cmd_vel`，本节点独占底盘串口。CMU 模式使用 `cmu_path_follower_generic` 产生导航速度，经命令适配与门控后交给本节点。

原包许可证字段为 `TODO`，来源许可证有待确认。实际串口设备和运动限值由部署配置提供。
