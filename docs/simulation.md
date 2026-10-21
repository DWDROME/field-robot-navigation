# 仿真环境

Gazebo Harmonic 仿真环境位于 upgrade/ros2，包含玉米田、果园、丘陵场地，Jackal/Husky 地面车，以及可选 PX4 X500 空中巡检。地面车复用升级版的定位、地形、FAR、CMU/MPPI 和命令门控。

```bash
git switch upgrade/ros2
# 按该分支 docs/simulation.md 准备固定依赖、构建镜像并启动场景。
```

仿真源码和世界在所属 ROS 包内，镜像与 Compose 在 docker 下。运行入口为该分支 `tools/run.sh navigation simulation`，构建和容器参数以该分支说明为准。

main 保留原版 ROS 1 实车链，当前没有与这套 Harmonic 环境连接的 ROS 1 适配。仿真任务完成情况与环境可启动情况在升级版文档中分别说明。
