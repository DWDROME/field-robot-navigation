# visibility_graph_msg — 可视图消息定义

**归属项目**: 项目 B — 温室自主导航系统（FAR Planner 生态）
**标签**: `ROS Msg` `Visibility Graph` `far_planner`

---

## 消息定义

为 FAR Planner 可视图算法提供 ROS 消息通信格式。

### Graph.msg

```
Header header
uint16 robot_id
visibility_graph_msg/Node[] nodes
uint32 size
```

可视图的完整序列化表示，包含图中所有节点。供 `far_planner` 发布 `/way_point` 和 `/navigation_boundary` 时使用。

### Node.msg

```
Header header
uint32 id
uint8 FreeType
geometry_msgs/Point position          # 节点空间位置
geometry_msgs/Point[] surface_dirs    # 表面法线方向
bool is_covered                       # 是否已被覆盖
bool is_frontier                      # 是否为前沿节点
bool is_navpoint                      # 是否为导航点
bool is_boundary                      # 是否为边界节点
uint32[] connect_nodes                # 邻接节点 ID 列表
uint32[] poly_connects                # 多边形连接
uint32[] contour_connects             # 轮廓连接
uint32[] trajectory_connects          # 轨迹连接
```

### 在管线中的位置

```
far_planner → /way_point (visibility_graph_msg/Graph) → graph_decoder / local_planner
far_planner → /navigation_boundary (visibility_graph_msg/Graph) → local_planner
```

### 依赖

- `message_generation` / `message_runtime`
- `std_msgs`, `geometry_msgs`
