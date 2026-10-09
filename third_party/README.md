# 第三方来源

本库维护的 ROS 包放在 `src/`；外部源码版本由四个互不重复的清单管理：

| 清单 | 组件 |
| --- | --- |
| [common.repos](common.repos) | Livox-SDK2、GTSAM |
| [ros1.repos](ros1.repos) | Sophus、vikit |
| [ros2.repos](ros2.repos) | FAST-LIO ROS 2 mapper、CMU autonomy stack/FAR、PCD localization、NDT-OMP、serial、virtual_maize_field、px4_msgs |
| [px4.repos](px4.repos) | PX4-Autopilot（v1.17.0）、Micro-XRCE-DDS-Agent（v2.4.3） |

```bash
python3 tools/prepare-dependencies.py ros2 .tmp/ros2-deps
bash tools/prepare-px4.sh .tmp/px4-deps
```

需要 Git 和 PyYAML。目标目录须尚不存在；可在目录参数后指定所需组件名。脚本按固定提交获取源码、初始化 submodule，再依次应用 `patches/<组件名>/*.patch`。已有修改保留在原工作区，脚本不会重置现有目录。`prepare-px4.sh` 单独处理 PX4 submodule，只递归初始化 SITL 路径需要的 MAVLink、PX4-gazebo-models、Micro-XRCE-DDS-Client、libevents 和 GPS devices；revision 均由固定父仓提交锁定，构建产物保存实际 `source-revisions.txt`。

FAST-LIO mapper 的补丁接入 [共享地图与回环核心](../src/localization/fast_lio_super/README.md)。CMU 补丁包含 Jazzy API、QoS、时间语义和有序全局路径发布；补丁主题与正文说明具体改动。

## 仿真来源与署名

| 组件或实际使用资源 | 固定来源 | 许可与署名 |
| --- | --- | --- |
| virtual_maize_field 生成器 | [FieldRobotEvent](https://github.com/FieldRobotEvent/virtual_maize_field/tree/e525d70c3c73fcdfae9c77bb8e4ca03bf3377ac3)，`e525d70c3c73fcdfae9c77bb8e4ca03bf3377ac3` | GPL-3.0；2021 Farm Technology Group of Wageningen University & Research、Kamaro Engineering e.V. |
| maize_01、maize_02 模型 | 同一固定源码的 `models/maize_01`、`models/maize_02` | CC BY-SA 4.0；2021 Kamaro Engineering e.V. |
| 草地纹理 | 同一源码 `Media/models/materials/textures` | CC0-1.0；2018 CC0Textures.com |
| 果园/丘陵 world、mesh，Clearpath 平台描述 | [固定 Debian 包版本](../docker/config/simulation.packages)，含 `clearpath-gz=2.9.4-1noble.20261006.043800` | Clearpath Robotics；clearpath_gz manifest 声明 BSD，上游 [LICENSE](https://github.com/clearpathrobotics/clearpath_simulator/blob/199464f0051d4e3cbffe4c744188367a39a38df6/LICENSE) 为 BSD-3-Clause（2023 clearpathrobotics）；各资源版权头保留 |
| PX4-Autopilot v1.17.0 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | BSD-3-Clause，PX4 contributors |
| PX4-gazebo-models | 父仓固定子模块 `b6127f4ec20de867e215fb5f78ae88b80f371909` | 仓库 BSD-3-Clause，2022 PX4 Autopilot for Drones；X500/OakD 模型各自 BSD-3-Clause，2022 Rudis Laboratories |
| px4_msgs release/1.17 | `86d8239e962f6939e05c3737784f60c02fa884db` | BSD-3-Clause，PX4 contributors |
| Micro-XRCE-DDS-Agent v2.4.3 | `73622810d984349b80bbac0ef55fc0b694d62222` | Apache-2.0，eProsima |
| pymavlink | SIM 专用 Python 环境固定 `2.4.50` | ArduPilot project；[COPYING](https://github.com/ArduPilot/pymavlink/blob/v2.4.50/COPYING) 声明 (L)GPL v3，并另列生成代码的 MIT 例外 |

生成式玉米田在构建时预生成，运行时不下载资源。固定任务只引用上述两种公开玉米模型，不使用未公开蒲公英；杂草、垃圾关闭。生成器许可与模型/纹理许可分别登记，不能统称 GPL。果园/丘陵的实际镜像输入是固定 Debian 软件包，不把侦察提交标为本机克隆来源；镜像几何报告保存实际 world、mesh、模型和航线配置 SHA256。PX4 v1.17 与 Agent v2.4.3 配套，v3.x Agent 不兼容。

FAR 补丁 [0004](patches/autonomy_stack_go2/0004-use-current-odometry-for-goal-reevaluation.patch) 修复首个目标到达、traversability 定时更新尚未执行时的空 odometry 节点指针：goal reevaluation 使用本次有效 odometry，并显式处理空值。该修复保持原规划几何与可达性条件，不把未知区域视为空闲。

FAR 补丁 [0005](patches/autonomy_stack_go2/0005-initialize-with-observed-free-terrain.patch) 允许实际观测到的空闲地形初始化图。上游只在障碍点非空时初始化，使没有障碍的已测地头永久处于 not-ready；本轮诊断实际收到 1695 个空闲点、零障碍点。补丁仍要求非空实测地形，不改变未知区、障碍阈值、footprint 或规划可达性规则。

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
