# 第三方来源

本库维护的 ROS 包放在 `src/`；外部源码版本由三个互不重复的清单管理：

| 清单 | 组件 |
| --- | --- |
| [common.repos](common.repos) | Livox-SDK2、GTSAM |
| [ros1.repos](ros1.repos) | Sophus、vikit |
| [ros2.repos](ros2.repos) | FAST-LIO ROS 2 mapper、CMU autonomy stack/FAR、PCD localization、NDT-OMP、serial |

```bash
python3 tools/prepare-dependencies.py ros2 .tmp/ros2-deps
```

需要 Git 和 PyYAML。目标目录须尚不存在；可在目录参数后指定所需组件名。脚本按固定提交获取源码、初始化 submodule，再依次应用 `patches/<组件名>/*.patch`。已有修改保留在原工作区，脚本不会重置现有目录。

FAST-LIO mapper 的补丁接入 [共享地图与回环核心](../src/localization/fast_lio_super/README.md)。CMU 补丁包含 Jazzy API、QoS、时间语义和有序全局路径发布；补丁主题与正文说明具体改动。

`autonomy_stack_go2` 的固定提交用于构建 `terrain_analysis_ext`、`visibility_graph_msg`、`far_planner`、`graph_decoder` 和 `boundary_handler`。基础地形、局部规划和通用 follower 由本库 `src/` 中的三个包维护；其来源见 [系统架构](../docs/architecture.md)。

PCD localization 和 NDT-OMP 保留固定来源与补丁供后续开发；默认 production 镜像不获取、编译或安装这两个组件，PCD profile 尚未接通整车运行。

| 本库维护组件 | 上游及维护入口 |
| --- | --- |
| FAST-LIO2 / OctVox | [FAST-LIO](https://github.com/hku-mars/FAST_LIO)、[Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)；`src/localization/fast_lio_super`、`slam_octvox` |
| FAST-LIVO2 | [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)；[完整源码与内存配置](../src/localization/fast_livo/README.md) |
| Livox ROS Driver 2 | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)；[ROS 1/ROS 2 共用源码](../src/drivers/livox_ros_driver2/README.md) |
| CMU 地形与局部导航 | [Autonomous Exploration](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)；`src/perception/terrain_analysis`、`src/navigation/cmu_*`，精确 fork 基准尚未确认 |
| Nav2 MPPI | [navigation2](https://github.com/ros-navigation/navigation2)；Jazzy 包名见 `docker/config/build.packages`、`runtime.packages` |

各包 LICENSE、版权头与 NOTICE 保留各自效力。CMU 路径数据由局部规划器运行时读取，随包保存。FAST-LIVO2 的上游许可声明差异及依赖原文保存在其 `provenance/`；详见包内来源说明。

CMU 官网目前链接上述 Autonomous Exploration 仓库，其 GitHub 父仓为 [ground_based_autonomy_basic](https://github.com/jizhang-cmu/ground_based_autonomy_basic)。上游 `local_planner/package.xml` 声明 BSD；本地三包的完整许可文本、原始版权与具体 BSD 条款仍待核实，现有 manifest 字段不代表已完成再分发许可核对。
