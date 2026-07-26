# dlrobot_robot — 履带底盘串口执行

**归属项目**: 项目 B — 温室自主导航系统
**标签**: `C++11` `serial` `URDF` `udev` `底盘控制`

---

## 技术核心

接收 `/cmd_vel` (geometry_msgs/Twist)，通过串口发送给底盘 MCU。

### 可执行文件

```cpp
// dlrobot_robot_node (C++11)
// src/dlrobot_robot.cpp
// - 订阅 /cmd_vel
// - serial 库打开串口设备 (如 /dev/ttyUSB0, 波特率 115200)
// - 将 Twist 转换为底盘协议：[speed, steer_angle] → 串口 byte stream
// - 包含异常重连机制

// src/Quaternion_Solution.cpp
// - 姿态四元数解算辅助
```

### 硬件配置

```bash
# scripts/dlrobot_udev.sh — 串口 udev 规则
# 配置底盘串口设备权限，确保 roslaunch 时无需 sudo

# urdf/ — 底盘 URDF 模型
# 供 robot_state_publisher 发布 TF 树，RViz 可视化
```

### Launch

```xml
<!-- launch/base_serial.launch -->
<node pkg="dlrobot_robot" type="dlrobot_robot_node" name="dlrobot_robot_node" output="screen"/>
```

### 在管线中的位置

```
导航管线末端：
  local_planner → /cmd_vel_stamped
    → cmd_vel_stamped_bridge → /cmd_vel (Twist)
      → dlrobot_robot_node → 串口 → 底盘 MCU
```
