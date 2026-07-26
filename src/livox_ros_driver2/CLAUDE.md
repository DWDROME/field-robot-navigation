# livox_ros_driver2 — Mid360 雷达驱动

**归属项目**: 项目 B — 温室自主导航系统
**定位**: Livox 官方 ROS 驱动 v2。接入 Mid360 LiDAR。

> **许可证**: BSD-2（Livox 官方）
> **语言**: C/C++

---

## 1. 对此模块的改动

此包为 Livox 官方发布，你不做内部修改。集成工作包括：

- 确认传感器安装外参不在此配置（收敛到 `greenhouse_nav`）
- 在 `greenhouse_nav` launch 中串联启动
- 注意此驱动使用 **CustomMsg** 类型（非标准 PointCloud2），`FAST_LIO`/`FAST_LIO_SUPER` 的 preprocess.cpp 相应处理

## 2. 结构

```
launch_ROS1/          # ROS1 启动配置
launch_ROS2/          # ROS2 启动配置（官方同时支持）
3rdparty/             # 依赖的第三方库（含 livox_sdk）
config/               # LiDAR 参数配置
msg/                  # CustomMsg 定义
```

## 3. Topic

- 发布: `/livox/lidar` (CustomMsg) — 非重复扫描点云
- 发布: `/livox/imu` (Imu) — 内置 IMU 数据

## 4. 构建

与 workspace 一起编译。支持 ROS1 Noetic + ROS2（Humble/Jazzy）双编译。
