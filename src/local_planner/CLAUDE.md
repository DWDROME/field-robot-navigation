# local_planner — CMU Local Planner + Path Follower

**归属项目**: 项目 B — 温室自主导航系统
**标签**: `C++` `Local Planning` `Path Following` `Trajectory Rollout`

---

## 技术核心

两个独立可执行文件构成局部导航管线：

### localPlanner

CMU 生态的局部路径规划器，基于：
- 订阅全局路径点 `/way_point` + 地形图 `/terrain_map`
- 生成局部可行路径 `/path`（避障 + 地形约束）
- 输出给 pathFollower

### pathFollower

路径跟踪器：
- 订阅局部路径 `/path` + 当前位姿 `/state_estimation`
- 输出 `/cmd_vel_stamped`（带时间戳的 Twist 指令）

### 正反向模式

通过 `greenhouse_nav` launch 中 TF 树的 yaw 偏移控制：

```xml
<!-- 正向（forward）：body → base_link yaw = 0 -->
<node pkg="tf2_ros" type="static_transform_publisher" name="body_to_baselink">
  <param name="frame_id" value="body"/>
  <param name="child_frame_id" value="base_link"/>
</node>

<!-- 反向（reverse）：body → base_link yaw = π -->
<!-- 通过 rosparam 传入 transform = [0, 0, 0, 0, 0, 3.14159] -->
```

### 管线位置

```
local_planner/
├── src/localPlanner.cpp     → 生成 /path
├── src/pathFollower.cpp     → 生成 /cmd_vel_stamped
├── launch/local_planner.launch
└── paths/                   → 预存参考路径

下游:
  pathFollower → /cmd_vel_stamped
    → greenhouse_nav/cmd_vel_stamped_bridge.py → /cmd_vel (Twist)
    → dlrobot_robot (串口底盘)
```
