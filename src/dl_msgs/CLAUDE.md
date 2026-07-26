# dl_msgs — dlrobot 底盘自定义消息定义

**归属项目**: 项目 B — 温室自主导航系统
**标签**: `ROS Msg` `Srv` `底盘协议`

---

## 消息列表

14 种自定义消息 + 1 个服务，覆盖底盘全部传感器与执行器：

### 传感器状态

| 消息 | 字段 | 用途 |
|------|------|------|
| `Battery.msg` | voltage, current, percentage | 电池电压/电流/百分比 |
| `Imu.msg` | orientation, angular_velocity, linear_acceleration | 底盘 IMU 数据 |
| `DHT22.msg` | temperature, humidity | 温湿度传感器 |
| `Sonar.msg` | range | 声纳测距 |
| `Ultrasonic.msg` | range | 超声波测距 |
| `Infrared.msg` | range | 红外测距 |
| `Ip.msg` | ip_address | 底盘模块 IP 地址 |

### 执行器控制

| 消息 | 字段 | 用途 |
|------|------|------|
| `Velocities.msg` | linear_x, angular_z | 速度指令 |
| `Servo.msg` | angle | 舵机角度 |
| `Arm.msg` | joint_angles | 机械臂关节 |
| `Led.msg` | mode, color, brightness | LED 控制 |
| `Buzzer.msg` | frequency, duration | 蜂鸣器控制 |
| `Relay.msg` | state | 继电器开关 |
| `PID.msg` | p, i, d, target | PID 参数 |

### 调试

| 消息/服务 | 用途 |
|-----------|------|
| `MsgTutorial.msg` | 通用调试消息 |
| `RcMode.msg` | 遥控模式切换 |
| `ServoAngle.srv` | 舵机角度服务 |

### 依赖

- `message_generation` / `message_runtime`
- `std_msgs`, `geometry_msgs`
