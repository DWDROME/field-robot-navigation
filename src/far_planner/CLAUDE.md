# far_planner — FAR 全局/路线规划

**归属项目**: 项目 B — 温室自主导航系统
**标签**: `Visibility Graph` `A*` `C++14` `PCL` `OpenCV`

---

## 技术核心

基于 visibility graph 的全局路径规划器。以 `/cloud_registered` + `/terrain_map*` 构建障碍物轮廓图，在可视图上运行 A* 搜索，输出 `/way_point` 和 `/navigation_boundary`。

### 源码结构（14 header + 9 source）

```cpp
// === 数据结构层 (include/far_planner/) ===
node_struct.h      // GraphNode: 位置 (x,y,z) + g_cost + h_cost + parent
point_struct.h     // PointCloud 体素化辅助结构
grid.h             // 栅格地图: 坐标 ←→ grid_index 映射
intersection.h     // 线段求交 / 障碍物碰撞检测
time_measure.h     // 微秒级性能计时

// === 核心算法 ===
contour_detector.h/cpp  // 点云 → 障碍物轮廓提取（Canny-like 边缘检测）
contour_graph.h/cpp     // 轮廓 → 可视图拓扑（Delaunay triangulation 插值）
dynamic_graph.h/cpp     // 动态图维护：插入/删除节点 + 边聚合
graph_planner.h/cpp     // A* / Dijkstra 在可视图上搜索最优路径
terrain_planner.h/cpp   // 地形感知 cost 函数（坡度 + 粗糙度 → traversal cost）

// === IO / 可视化 ===
map_handler.h/cpp       // 点云地图 → 内部 costmap 转换
scan_handler.h/cpp      // 激光扫描数据累积与管理
planner_visualizer.h/cpp // RViz MarkerArray 可视化（边 / 路径 / 轮廓）
graph_msger.h/cpp       // 可视图序列化 / ROS 消息通信
utility.h/cpp           // 数学工具（角度归一化、距离计算）
far_planner.h/cpp       // 主类：整合以上所有模块
```

### Topic 合约

| 方向 | Topic | 类型 | 消费方 |
|------|-------|------|--------|
| 订阅 | `/cloud_registered` | sensor_msgs/PointCloud2 | 地图构建 |
| 订阅 | `/terrain_map` | sensor_msgs/PointCloud2 | 地形 cost |
| 订阅 | `/terrain_map_ext` | sensor_msgs/PointCloud2 | 扩展地形层 |
| 订阅 | `/goal_point` | geometry_msgs/PointStamped | 导航目标 |
| 发布 | `/way_point` | visibility_graph_msg/... | local_planner |
| 发布 | `/navigation_boundary` | visibility_graph_msg/... | local_planner |

### 依赖

- `visibility_graph_msg` — 自定义消息定义
- PCL (≥1.8) — 点云处理
- Eigen3 — 线性代数
- OpenCV — 图像形态学操作（contour_detector 中用于膨胀/腐蚀）

### 在管线中的位置

```
terrain_map* + goal_point → far_planner
  → visibility graph A*
  → way_point + navigation_boundary → local_planner
```
