# greenhouse_nav — 温室导航系统集成

**归属项目**: 项目 B — 温室自主导航系统
**标签**: `ROS1 Noetic` `Launch 装配` `TF 树` `Python` `mid360 外参补偿`

---

## 1. 集成范围

温室导航主线的用户入口收敛、launch 装配、传感器外参补偿、定位管理、速度指令桥接。

### 1.1 传感器外参补偿：Mid360 前下倾 27°

**物理背景**：Mid360 安装时绕 Y 轴向前倾斜 27°，天然坐标系倾斜。若保留 FAST-LIO 原生 `camera_init` 帧，RViz 中看到的 `/cloud_registered` 是倾斜的。

**技术方案** — 不修改 FAST-LIO 内部坐标语义，额外发布水平辅助帧：

```xml
<!-- system_greenhouse_base.launch -->
<!-- 局部水平补偿：body → body_foot -->
<node pkg="tf2_ros" type="static_transform_publisher" name="body_to_body_foot">
  <param name="frame_id"   value="body"/>
  <param name="child_frame_id" value="body_foot"/>
  <!-- 绕 X 轴旋转 +27° 取消前倾 (即绕 Y 轴 -27° = -0.471239 rad) -->
  <rosparam param="quaternion">[0, -0.233445, 0, 0.972370]</rosparam>
</node>

<!-- 世界帧水平化：camera_init → robot_foot_init -->
<node pkg="tf2_ros" type="static_transform_publisher" name="camera_init_to_robot_foot_init">
  <param name="frame_id"   value="camera_init"/>
  <param name="child_frame_id" value="robot_foot_init"/>
  <rosparam param="quaternion">[0, -0.233445, 0, 0.972370]</rosparam>
</node>

pitch = -27° = -0.471239 rad（绕 Y 轴）
旋转四元数: q = [0, -sin(θ/2), 0, cos(θ/2)]
            = [0, -0.233445, 0, 0.972370]
```

**设计决策** — 外参不塞进：
- `livox_ros_driver2` 的 `extrinsic_parameter`
- `FAST_LIO/config/mid360.yaml` 的 `extrinsic_R/T`

而是唯一收敛到 `system_greenhouse_base.launch` + `localization_manager.py`。

**效果**：
- 建图入口默认 `Fixed Frame = robot_foot_init`（水平）
- `/state_estimation` 仍用 `base_link`，下游导航不受影响

### 1.2 Launch 装配体系

**4 个用户入口**：

```xml
<!-- greenhouse_mapping.launch — 建图模式 -->
<include file="$(find livox_ros_driver2)/launch_ROS1/livox_lidar.launch"/>
<include file="$(find fast_lio)/launch/mapping_mid360.launch"/>
<!-- 注：此处用 fast_lio，如需切换到 fast_lio_super 只需改 include -->

<!-- greenhouse_navigation.launch — 导航模式（正向） -->
<include file="$(find greenhouse_nav)/launch/system_greenhouse_base.launch"/>
<include file="$(find livox_ros_driver2)/launch_ROS1/livox_lidar.launch"/>
<include file="$(find fast_lio)/launch/mapping_mid360.launch"/>
<include file="$(find fast_lio_localization)/launch/localization_avia.launch"/>
<include file="$(find terrain_analysis)/launch/terrain_analysis.launch"/>
<include file="$(find terrain_analysis_ext)/launch/terrain_analysis.launch"/>
<include file="$(find far_planner)/launch/far_planner.launch"/>
<include file="$(find local_planner)/launch/local_planner.launch"/>
<node pkg="greenhouse_nav" type="cmd_vel_stamped_bridge.py" name="cmd_vel_stamped_bridge"/>
<include file="$(find dlrobot_robot)/launch/base_serial.launch"/>

<!-- greenhouse_navigation_forward.launch → body→base_link yaw = 0 -->
<!-- greenhouse_navigation_reverse.launch → body→base_link yaw = π -->
```

正反向模式差异仅在 TF 树中 `body → base_link` 的 yaw 旋转（0 vs π rad）。

**内部入口**（供回放/module-level 调试）：

| launch | 用途 |
|--------|------|
| `system_greenhouse_base.launch` | 外参 + 辅助帧 + 基础传感器 |
| `system_greenhouse_with_route_planner.launch` | 完整装配含 route planner |
| `system_greenhouse_bag.launch` | bag 回放模式 |

## 2. 关键脚本

### localization_manager.py

Python 定位管理器，负责：
- 接收 FAST-LIO 的 `/Odometry`（高频，局部一致）
- 接收 FAST_LIO_LOCALIZATION 的全局定位结果（低频，0.5-0.2 Hz）
- 融合输出 `/state_estimation`（map→base_link）
- 管理坐标帧重映射

### cmd_vel_stamped_bridge.py

```python
#!/usr/bin/env python3
# 订阅: /cmd_vel_stamped (geometry_msgs/TwistStamped)
# 发布: /cmd_vel      (geometry_msgs/Twist)
# 功能: 剥离时间戳，向下游底盘节点兼容

def callback(msg):
    cmd = Twist()
    cmd.linear  = msg.twist.linear
    cmd.angular = msg.twist.angular
    pub.publish(cmd)
```

## 3. Topic 合约

| Topic | 类型 | 方向 | 连接 | 频率 |
|-------|------|------|------|------|
| `/livox/lidar` | livox_ros_driver2/CustomMsg | sensor→system | Mid360 → FAST-LIO/SUPER | 10 Hz |
| `/livox/imu` | sensor_msgs/Imu | sensor→system | Mid360 → FAST-LIO/SUPER | 200 Hz |
| `/Odometry` | nav_msgs/Odometry | internal | FAST-LIO → localization_manager | ∼100 Hz |
| `/cloud_registered` | sensor_msgs/PointCloud2 | internal | FAST-LIO → far/terrain | 10 Hz |
| `/state_estimation` | nav_msgs/Odometry | system→planner | localization_manager → local_planner | ∼100 Hz |
| `/terrain_map` | sensor_msgs/PointCloud2 | internal | terrain_analysis → far/local | 10 Hz |
| `/terrain_map_ext` | sensor_msgs/PointCloud2 | internal | terrain_analysis_ext → far | 10 Hz |
| `/goal_point` | geometry_msgs/PointStamped | user→system | user → far_planner | 事件 |
| `/way_point` | visibility_graph_msg/... | internal | far_planner → local_planner | 1-10 Hz |
| `/navigation_boundary` | visibility_graph_msg/... | internal | far_planner → local_planner | 1 Hz |
| `/path` | nav_msgs/Path | internal | local_planner → (visualization) | 10 Hz |
| `/cmd_vel_stamped` | geometry_msgs/TwistStamped | internal | pathFollower → bridge | 10-50 Hz |
| `/cmd_vel` | geometry_msgs/Twist | system→actuator | bridge → dlrobot_robot | 10-50 Hz |

## 4. 标准工作流

```
# 建图
roslaunch greenhouse_nav greenhouse_mapping.launch
# (采一圈环境后停止)
# PCD 自动保存到 src/FAST_LIO/PCD/scans.pcd

# 导航（正向模式）
roslaunch greenhouse_nav greenhouse_navigation.launch
# 发送初值
rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
# 发送导航目标
rostopic pub -1 /goal_point geometry_msgs/PointStamped \
  '{header: {frame_id: "map"}, point: {x: 3.0, y: 0.0, z: 0.0}}'
```

## 5. 文档参考

- `README.md` — 使用说明
- `.agentdocs/计划书.md` — 温室 V1 最小闭环任务流程
- `.agentdocs/执行计划书.md` — 实施拆解
- `.agentdocs/命令索引.md` — 所有 roslaunch/rosrun 命令索引
