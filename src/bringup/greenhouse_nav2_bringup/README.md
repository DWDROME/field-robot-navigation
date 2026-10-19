# greenhouse_nav2_bringup

ROS 2 Jazzy 系统启动包，负责配置校验、话题/TF/设备合同和健康监控。

## 闭环导航

```bash
ros2 launch greenhouse_nav2_bringup closed_loop.launch.py deployment:=/absolute/deployment.yaml
```

该入口装配 FAST-LIO、后台回环、FAR 和航点任务。部署模板默认使用 CMU local planner/follower，`controllers: [mppi]` 选择 MPPI；两模式共享部署校验与命令门控。配置字段与操作见 [导航说明](../../navigation/greenhouse_mppi_navigation/README.md)。

## 建图与 CMU 配置

`bringup.launch.py` 提供独立 profile 入口：

```bash
ros2 run greenhouse_nav2_bringup greenhouse_validate_bringup \
  --config-dir "$(ros2 pkg prefix greenhouse_nav2_bringup)/share/greenhouse_nav2_bringup/config" \
  --profiles mapping-fastlio --host-profile simulation --require-runtime-ready
ros2 launch greenhouse_nav2_bringup bringup.launch.py \
  profiles:=mapping-fastlio,navigation-far host_profile:=simulation
```

`start_runtime_nodes:=false` 只解析配置并按需启动监控；`plan_output:=/path/plan.json` 保存解析结果。FAR 配置支持 `simulation` 和 `dev-ci`，由校验器拒绝互斥定位源、重复导航控制权及未实现配置。仿真参数需要外部时钟和传感器，真实设备通过闭环导航的显式部署入口配置。
