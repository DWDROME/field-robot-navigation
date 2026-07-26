# livox_to_pointcloud2 — Livox CustomMsg to PointCloud2 转换

**归属项目**: 项目 B — 温室自主导航系统
**标签**: `C++` `PCL` `Livox` `CustomMsg` `PointCloud2`

---

## 技术核心

将 `livox_ros_driver2/CustomMsg` 类型转换为标准 `sensor_msgs/PointCloud2`，供需要标准点云类型的下游节点使用。

### 可执行文件

```
src/livox_repub.cpp → livox_to_pointcloud2 节点
```

- 订阅 `/livox/lidar` (CustomMsg)
- 解析 Livox 自定义点格式（包含反射率、线号、OffsetTime 等）
- 发布为 `sensor_msgs::PointCloud2`（`pcl::PointXYZINormal` 类型）
- 支持多帧累积合并（`TO_MERGE_CNT` 参数）

### 用途

当下游模块（如 PCL 算法、Open3D 处理）需要标准 PointCloud2 输入时，此节点提供桥接转换。当前导航主线中 FAST-LIO 原生直接消费 CustomMsg，此节点作为备用/调试组件。

### 依赖

- `livox_ros_driver2` — CustomMsg 定义
- PCL (≥1.8) — 点云数据结构
- ROS1 Noetic

### 管线位置

```
livox_ros_driver2 → /livox/lidar (CustomMsg)
  → livox_to_pointcloud2 → /livox/cloud (PointCloud2)
  → 下游标准 PCL 节点
```
