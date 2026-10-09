# ROS 2 Jazzy 容器

`Dockerfile` 构建生产镜像：Livox、FAST-LIO、GTSAM 回环、地形分析、FAR、CMU、MPPI、命令门控与底盘驱动。默认部署模板使用 CMU，MPPI 由配置选择。各 builder 共用构建依赖环境，按真实依赖复制编译结果。

PCD 定位保留外部源码登记，供后续开发；当前镜像不编译或安装 PCD，其预留 profile 尚未接通整车运行。

`Dockerfile.sim` 基于生产镜像添加 Gazebo、Clearpath A200 及仿真资源。源码和机器人参数归所属 ROS 包；外部 Git 版本与补丁见 [third_party](../third_party/README.md)。基础镜像 digest 直接保存在 `Dockerfile`，`config/*.packages` 按构建、运行和仿真列出 APT 包名。

## 构建

```bash
bash tools/build-ros2.sh
```

交付平台为 amd64。可通过 `HTTP_PROXY`、`HTTPS_PROXY`、`NO_PROXY` 设置构建网络；使用镜像自身的软件源获取当前可安装版本。构建不读取机器人部署目录或底盘设备。

## 运行

准备配置目录，将 [部署模板](../src/navigation/greenhouse_mppi_navigation/config/deployment-hardware.template.yaml) 保存为 `deployment.yaml`，填写设备、网络、外参、footprint、运动限制和急停接口，并放入 Livox JSON。配置中的容器路径使用 `/config/...`，底盘路径与 `CHASSIS_DEVICE` 一致。

```bash
export DEPLOYMENT_DIRECTORY=/absolute/robot-config
export CHASSIS_DEVICE=/dev/serial/by-id/your-chassis
docker compose -f docker/compose.yaml -f docker/compose.hardware.yaml config --quiet
docker compose -f docker/compose.yaml -f docker/compose.hardware.yaml up closed-loop
```

`compose.hardware.yaml` 显式挂载部署目录与底盘设备。软件配置检查在启动时执行；真实标定与急停有效性须在实际平台确认。任务发送和取消见 [导航操作](../src/navigation/greenhouse_mppi_navigation/README.md)。

同一镜像可运行 [bringup 的独立建图与 CMU profile](../src/bringup/greenhouse_nav2_bringup/README.md)，每次选择一套命令控制源。需要集中 DDS 发现时可启动 `discovery` profile，并在参与节点环境设置 `ROS_DISCOVERY_SERVER` 与 Fast DDS participant 配置。

## 仿真

仿真镜像在 production 之上加入 Gazebo、Clearpath 平台、虚拟玉米田与 PX4 SITL，构建分四步（准备固定源码、production 镜像、PX4 构建、仿真镜像）：

```bash
bash tools/prepare-simulation.sh
docker compose -f docker/compose.sim.yaml build production
bash tools/build-px4.sh
docker compose -f docker/compose.sim.yaml build simulation
```

运行三场地 × 三机器人矩阵的单场景入口：

```bash
docker compose -f docker/compose.sim.yaml up simulation
SIM_WORLD=pipeline SIM_ROBOT=jackal_j100 docker compose -f docker/compose.sim.yaml up simulation
```

`SIM_WORLD` 选择 maize_field / orchard / pipeline，`SIM_ROBOT` 选择 jackal_j100 / husky_a200 / x500；一个入口管理 world、实体生成与 `/clock`。批量九组合测试与截图见[仿真说明](../docs/simulation.md)。仿真世界、桥接配置与 launch 均在 `src/bringup/greenhouse_nav2_bringup/`，空中任务在 `src/bringup/greenhouse_sim_air/`。
