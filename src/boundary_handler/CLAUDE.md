# boundary_handler — 边界图提取器

**归属项目**: 项目 B — 温室自主导航系统（FAR Planner 生态）
**标签**: `C++` `Boundary Detection` `Visibility Graph` `PCL`

---

## 技术核心

从点云中提取可通行边界，构建 FAR Planner 的边界图拓扑。

### 可执行文件

```
src/graph_extractor.cpp → graph_extractor 节点
include/boundary_handler/graph_extractor.h  — 边界图提取主类
include/boundary_handler/intersection.h     — 线段求交辅助
include/boundary_handler/point_struct.h     — 点云数据结构
```

### 订阅/发布

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `/cloud_registered` | sensor_msgs/PointCloud2 |
| 订阅 | `/terrain_map` | sensor_msgs/PointCloud2 |
| 发布 | `/navigation_boundary` | visibility_graph_msg/Graph |
| 发布 | `/boundary_marker` | visualization_msgs/MarkerArray |

### 处理流程

1. 接收累积点云
2. 边界检测（Canny-like 轮廓提取）
3. 构建边界图拓扑（节点 + 边）
4. 发布给 local_planner 作为导航约束

### 在管线中的位置

```
cloud_registered + terrain_map → boundary_handler
  → navigation_boundary (Graph msg) → local_planner
```

### 依赖

- `visibility_graph_msg` — 可视图消息定义
- PCL (≥1.8) — 点云处理
- ROS1 Noetic
