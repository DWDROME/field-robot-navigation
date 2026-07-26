# dlrobot — 轮式差速底盘 ROS 驱动

**归属项目**: 项目 B — 温室自主导航系统（前代/参考底盘）
**标签**: `C++` `rosserial` `Differential Drive` `STM32`

---

## 技术核心

基于 `rosserial_python` 的轮式差速底盘 ROS 驱动，与 `dlrobot_robot` （串口直连）属不同实现方案。

### 可执行文件

```
src/dl_base.cpp       — 底层控制逻辑（速度解析 → 串口协议）
src/dl_base_node.cpp  — ROS 节点封装
include/dl_base.h     — 协议定义与接口
```

### 工作流

1. 订阅 `/cmd_vel` (Twist)
2. 通过 `rosserial_python` 桥接与 STM32 MCU 通信
3. 差速解算：`speed_L = v - w * wheel_base / 2`, `speed_R = v + w * wheel_base / 2`
4. 发布底盘状态（电压、里程计等）通过 `dl_msgs` 自定义消息

### 依赖

- `rosserial_python` — ROS ↔ 串口桥接
- `dl_msgs` — 底盘自定义消息定义（Arm, Battery, PID, Velocities 等 14 种）
- `tf` — 坐标变换

### 与 dlrobot_robot 的区别

| 维度 | dlrobot | dlrobot_robot |
|------|---------|---------------|
| 通信方式 | rosserial_python (serial bridge) | C++ serial 库直连 |
| 语言 | C++ + Python | C++11 |
| 用途 | 前代方案，保留参考 | 当前导航主线末端执行器 |
| License | BSD | 未标注 |
