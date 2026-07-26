# graph_decoder — 可视图解码器

**归属项目**: 项目 B — 温室自主导航系统（FAR Planner 生态）
**标签**: `C++` `Visibility Graph` `PCL` `far_planner`

---

## 技术核心

将 `visibility_graph_msg/Graph` 消息解码回可视图数据结构，为下游节点提供图拓扑信息。

### 可执行文件

```
src/decoder_node.cpp → decoder_node
include/graph_decoder/decoder_node.h    — 解码器主类
include/graph_decoder/point_struct.h    — 点云辅助结构
```

### 订阅/发布

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `/visibility_graph` | `visibility_graph_msg/Graph` |
| 订阅 | `/way_point` | `visibility_graph_msg/Graph` |
| 发布 | `/decoded_graph` | 解码后的图可视化数据 |
| 发布 | `/decoded_path` | 解码后的路径 |

### 在管线中的位置

```
far_planner → /way_point (Graph msg)
  → graph_decoder → 解码为路径点序列 → local_planner
```

### 依赖

- `visibility_graph_msg` — 可视图消息定义
- PCL (≥1.8) — 点云数据处理
- ROS1 Noetic
