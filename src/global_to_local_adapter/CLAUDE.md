# global_to_local_adapter — 全局→局部路径适配器

**归属项目**: 项目 B — 温室自主导航系统（预留未启用）
**定位**: 将 FAR Planner 的全局路径转换为 local_planner 可消费的格式。

> **许可证**: 未标注
> **语言**: Python

---

## 1. 保留原因

你在启动集成时保留了此包但不启用。设计上它提供了全局路径到局部路径的格式转换层。

## 2. 当前状态

- **默认不启用** — 不在 `greenhouse_navigation.launch` 主线中激活
- 为后续替换/调试路径转换逻辑预留接口
- 无独立配置文件

## 3. Topic

- 订阅: `/way_point`（来自 far_planner）
- 发布: 本地化后的路径

## 4. 依赖

ROS1 Noetic, rospy
