# Robot Navigation & SLAM

`main` 是 ROS 1 Noetic 实车交付线：Livox MID360 点云经 FAST-LIO 建图，FAST-LIO_LOCALIZATION 在已有 PCD 地图中定位，MoveBase3D 做全局规划、地势分析与局部控制，`dlrobot_robot` 把速度指令写成底盘串口帧。18 个 ROS 包，保留原包名与算法源码。

> 状态：2026-10-21 完成仓库整理与静态核对，未编译、未运行、未上车。依据见 [docs/roadmap.md](docs/roadmap.md)。
> 另一条线是 `upgrade/ros2`（ROS 2 Jazzy、Gazebo/PX4、独立 FAST-LIVO2）；`firmware/tr800_f407` 两条线共用。

## 先读哪份

| 要知道 | 读 |
| --- | --- |
| 环境、依赖、构建 | [docs/build.md](docs/build.md) |
| 三种模式与参数 | [docs/modes.md](docs/modes.md) |
| 地图、配置、话题、串口帧 | [docs/data-formats.md](docs/data-formats.md) |
| 模块关系与 TF | [docs/architecture.md](docs/architecture.md) |
| 当前状态、待办、风险 | [docs/roadmap.md](docs/roadmap.md) |
| 固件构建、烧录、协议 | [firmware/tr800_f407/README.md](firmware/tr800_f407/README.md) |

## 构建

Ubuntu 20.04 + ROS Noetic。系统依赖 PCL、Eigen、OpenMP、OMPL、OctoMap、串口库；定位与规划另需 CasADi、Open3D、`ros_numpy`。Livox-SDK2 单独安装，版本与补丁见 [third_party/README.md](third_party/README.md)。

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash tools/build-ros1.sh          # catkin_make -DROS_EDITION=ROS1；BUILD_JOBS 默认 2
source devel/setup.bash
```

`build-ros1.sh` 只构建 `src/`，固件不参与 catkin。

## 运行

```bash
# 建图；Ctrl+C 正常退出后写入 src/FAST_LIO/PCD/scans.pcd
bash tools/run.sh mapping hardware

# 定位；必须给已有 PCD 地图
bash tools/run.sh localization hardware map:=/absolute/map.pcd

# 导航；定位 + 底盘 + MoveBase3D + 实时点云过滤
bash tools/run.sh navigation hardware map:=/absolute/map.pcd
```

`localization` 与 `navigation` 缺 `map:=` 时脚本直接报错。参数、初始位姿、二维辅助图和旧入口 `start_all.sh` 见 [docs/modes.md](docs/modes.md)。

## 仓库结构

```text
src/
  FAST_LIO/                 建图（FAST-LIO2）
  FAST_LIO_LOCALIZATION/    已有 PCD 地图定位
  MoveBase3D/               规划、地势分析、局部控制、整车启动（8 个包）
  livox_ros_driver2/        MID360 驱动（ROS1/ROS2 双配置）
  livox_to_pointcloud2/     点云转 PointCloud2
  pcd2pgm_package/          PCD 转二维栅格地图
  dlrobot_robot/            当前底盘串口入口
  dlrobot/、dl_msgs/        旧底盘包与消息定义
docs/                       架构、构建、模式、数据格式、状态
tools/                      run.sh 与 build-ros1.sh
firmware/tr800_f407/        下位机源码、构建与烧录
third_party/                Livox-SDK2 版本与补丁
```

## 常见问题

| 现象 | 处理 |
| --- | --- |
| 提示 `Provide the existing 3D map` | 补 `map:=/absolute/map.pcd` |
| `roslaunch: command not found` | 没有 source Noetic 或 `devel/setup.bash` |
| 串口打不开 | 设备名不对，或用户不在 `dialout` 组 |
| 雷达没有点云 | `MID360_config.json` 的 IP 与实际网络不符 |
| 地图没有保存 | 需要 Ctrl+C 正常退出，且 `src/FAST_LIO/PCD` 可写 |

仿真不在本分支：`run.sh ... simulation` 会提示切换 `upgrade/ros2`。

## 交接

- 两条分支各自完整，`firmware/` 实现相同，只有 README 不同。
- 地图与录包是运行数据，不入库；用 `map:=` 指向实际文件。
- `.worktrees/`、`.cache/`、`.tmp/` 是本地审计材料，不属于分支内容；删 `.worktrees` 会丢归档 worktree。
- 串口协议改动要同时改固件 `HARDWARE/usartx.h`、本线 `src/dlrobot_robot/include/dlrobot_robot.h` 与升级线驱动。
- 仓库没有 CI，改动后至少跑一次 `bash tools/run.sh --help`、相关包构建和 launch 语法检查。

## 来源与许可

原版包名、版权头、LICENSE 与 README 声明全部保留；第三方来源见 [third_party/README.md](third_party/README.md)。MoveBase3D 原始 README 注明“仅供科研测试，请勿商用”，其余包遵循各自条款。
