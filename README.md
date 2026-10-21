# 温室/履带车 ROS 2 导航

这是 `upgrade/ros2`：Jazzy 下的整车版本。Mid-360 的点云和 IMU 进 FAST-LIO，地图后端可选 ikd-Tree 或 OctVox，回环用 Scan Context + GICP + iSAM2；导航默认 FAR 出航点、CMU planner + follower 跟踪，也可以切到 Nav2 MPPI。Gazebo Harmonic 里有玉米田、果园、丘陵三个场地，可选 PX4 X500 空中巡检。FAST-LIVO2 单拎出来，还是 Noetic 工作区。

ROS 1 实车那套在 `main`，下位机 `firmware/tr800_f407` 两边共用。这个分支只负责 ROS 2。

车型同样只对应 **TR800 大履带**（`TR800_F407`）；TR500 小履带在固件文档里只是参考，没有适配。

## 编译

按目标三选一，命令都在仓库根目录跑：

```bash
# Jazzy 镜像，需要 BuildKit 和 Compose 2.17+，amd64
bash tools/build-ros2.sh

# Noetic 的 SLAM 工作区（fast_lio_super 等）
bash tools/build-ros1.sh .tmp/ros1-slam

# FAST-LIVO2，独立 Noetic 工作区
bash tools/build-fastlivo2.sh .tmp/fastlivo-build
```

Noetic 依赖先用脚本拉固定版本，再按各自 CMake 装：

```bash
python3 tools/prepare-dependencies.py ros1 .tmp/slam-deps gtsam livox_sdk2
```

工作区目录必须不存在，脚本会自己建。构建和测试的临时东西都丢 `.tmp/`，别提交。各入口的前置条件在 [docs/build.md](docs/build.md)。

## 实车

```bash
bash tools/run.sh navigation hardware deployment:=/abs/deployment.yaml
```

`deployment.yaml` 从 `src/navigation/greenhouse_mppi_navigation/config/deployment-hardware.template.yaml` 复制，里面 `null` 是故意的——没填全就拒绝启动，不会拿默认值凑合。`controllers` 只能 `[cmu]` 或 `[mppi]` 二选一，CMU 只判位置到达，要终点朝向就选 MPPI。

整车入口是 `greenhouse_nav2_bringup/closed_loop.launch.py`，起之前把传感器标定、底盘设备、限速和急停接口填好。

## 仿真

四步，缺一不可：

```bash
bash tools/prepare-simulation.sh
BUILD_JOBS=1 docker compose -f docker/compose.sim.yaml build simulation-build
BUILD_JOBS=1 bash tools/build-px4.sh
BUILD_JOBS=1 docker compose -f docker/compose.sim.yaml build simulation
```

运行必须叠加 `docker/compose.sim.runtime.yaml`，WSL2 再加 `docker/compose.sim.wsl.yaml`：

```bash
docker compose -f docker/compose.sim.yaml -f docker/compose.sim.runtime.yaml run --rm simulation \
  ros2 launch greenhouse_nav2_bringup simulation_matrix.launch.py \
  world:=maize_field robot:=husky_a200 headless:=true output:=/evidence/run1
```

九组合批量测试用 `run_matrix.py --runs 3`，参数、证据格式和已知问题在 [docs/simulation.md](docs/simulation.md)。

## 目录

```text
src/
  drivers/       雷达和底盘串口
  perception/    点云规范化、地形分析
  localization/  FAST-LIO、OctVox、FAST-LIVO2
  navigation/    FAR、CMU、MPPI、任务与航点
  control/       命令门控
  bringup/       整车装配、配置校验、仿真入口
docker/          镜像、Compose、运行配置
firmware/        下位机（和 main 同一份）
tools/           构建与运行入口
third_party/     外部源码版本与补丁
docs/            架构、构建、仿真、数据格式、状态
```

接口、QoS、profile、部署字段这些约定在 [docs/data-formats.md](docs/data-formats.md)；数据流和 TF 在 [docs/architecture.md](docs/architecture.md)。

## 现在能跑到哪

装配和合同检查做过，24 项 pytest、16 项入口分派都过，但**镜像没重建、矩阵没复跑、没上实车**。仿真有一轮 10-07 的记录：27 次运行 8 次通过，18 次地面任务全失败。原因是 VLP16 近场看不到地面，MPPI 拒绝轨迹，最后零速停机。这不是“仿真可用”，只是“链路能起来并安全停下”。别用放宽 unknown、缩 footprint 的方式把它做成通过。

## 坑和限制

- 升级版 PCD 定位没接，`localization` 入口会直接拒绝，不是配置错了。
- 通用 hardware profile 的纯建图也还没接，只支持 simulation/dev-ci。
- 底盘串口只发里程计和电压；遥测 8–19 是控制诊断分页，不是 IMU，导航的 `/sensors/imu/data` 来自 Livox。
- CMU 的 local Path 没有任务身份，取消 A 之后晚到的 A 路径理论上还能授权，长期跑之前要补 epoch/UUID。
- X500 只做航点巡检和感知展示，没有三维避障、没有地形跟随。
- 仿真容器和构建别并行跑，内存不够会 OOM；WSL 用 D3D12/EGL 配置，软件渲染达不到消息新鲜度要求。

## 许可

集成的 FAST-LIO、Super-LIO/OctVox、FAST-LIVO2、Livox SDK、CMU 导航、FAR、Nav2、GTSAM 都保留原许可声明，来源和版本锁在 [third_party/README.md](third_party/README.md)。
