# 仿真矩阵：三场地 × 三机器人

本说明覆盖 Gazebo Harmonic 仿真矩阵的准备、构建、单场景启动、批量测试与截图流程。仿真矩阵在 production 镜像之上由 `docker/Dockerfile.sim` 扩展：加入 Gazebo、Clearpath 平台、虚拟玉米田与 PX4 SITL。

| 场地 | 世界来源 | 机器人 | 说明 |
| --- | --- | --- | --- |
| 玉米田 maize_field | virtual_maize_field（ros2-gz，构建期固定 seed 预生成） | Jackal J100 / Husky A200 / X500 | 12 行直行+曲行作物、逐行缺株、地头、起伏与沟渠 |
| 果园 orchard | clearpath_gz `worlds/orchard.sdf`（apt 2.9.4） | 同上 | 成排树木、长通道、小坡 |
| 丘陵 pipeline | clearpath_gz `worlds/pipeline.sdf` | 同上 | 丘陵高差、河流、桥梁、洞穴、管线、太阳能设施 |

地面车（Jackal J100、Husky A200）以同一套仿真传感器（Velodyne VLP16 三维雷达、IMU、RGB-D 相机、平台轮式里程计）接入既有地形分析、FAR、CMU/MPPI 与命令门控链路；X500 以 PX4 SITL（v1.17.0，`x500_depth`）挂接同一运行世界，执行真实 Offboard 航点巡检。X500 的仿真范围是空中航点巡检与感知展示；未实现三维避障与地形跟随，相关结果不宣称支持。

## 准备与构建

固定来源由 `third_party/` 清单管理（virtual_maize_field、PX4-Autopilot、px4_msgs、Micro-XRCE-DDS-Agent 的完整 commit）。构建分四步：

```bash
# 1. 获取固定的仿真源码到 .tmp/simulation（玉米田、px4_msgs、PX4、DDS Agent）
bash tools/prepare-simulation.sh

# 2. 构建 PX4 工具链镜像（自动先构建 production）
BUILD_JOBS=1 docker compose -f docker/compose.sim.yaml build simulation-build

# 3. 构建 PX4 SITL 与 Micro-XRCE-DDS Agent 并暂存产物
BUILD_JOBS=1 bash tools/build-px4.sh

# 4. 构建仿真镜像
BUILD_JOBS=1 docker compose -f docker/compose.sim.yaml build simulation
```

首次构建需要访问镜像、Git 与软件包源；需要代理时先导出 `HTTP_PROXY`、`HTTPS_PROXY` 和 `NO_PROXY`。`BUILD_JOBS` 控制并行度。`tools/build-px4.sh` 使用独立的 `greenhouse-sim-build:local`，不依赖最终仿真镜像；构建容器默认限制为 4 GiB 内存、6 GiB 总内存与交换空间。产物暂存在 `.tmp/simulation/px4`，作为最终镜像构建上下文使用。构建与仿真应串行运行，避免宿主机内存耗尽。

PX4 编译在容器的 Linux 文件系统进行；准备源码以只读方式挂载，避免 NTFS 的 Git 所有权和构建 I/O 问题。脚本同时向 make 和 Ninja 传递 `BUILD_JOBS`，不依赖无 TTY 容器中的 `ps T` 自动探测。`source-revisions.txt` 随暂存产物及镜像保存。

需要保留其他任务的本地镜像标签时，可在构建和运行前设置独立标签：

```bash
export SIM_PRODUCTION_IMAGE=greenhouse-jazzy:gazebo-sim-matrix
export SIM_IMAGE=greenhouse-sim:gazebo-sim-matrix
```

Clearpath 世界和机器人描述使用 [simulation.packages](../docker/config/simulation.packages) 中固定的 Debian 版本；实际交付来源是这些软件包及资源哈希，不把侦察时的 Git 提交冒充为本机获取的源码。SIM 入口单独加载生产、FAR、玉米田及空中包前缀，并加载隔离的 `pymavlink==2.4.50`，不修改生产入口。

`px4_msgs` 与 Python 飞行任务分别构建并缓存到 SIM 专用前缀 `/opt/greenhouse_air`。SIM 层还安装自己的启动文件、仿真脚本、配置和世界；导航库仍来自 production。仅修改这些 SIM 文件时，可以用已编译的 production 镜像作为命名上下文重建 SIM，避免重复编译导航库。最终镜像的资源路径显式包含 Clearpath `meshes`、玉米模型与 PX4 模型；仅有世界 SDF 不能证明其碰撞和视觉资源成功加载。

运行命令必须叠加 `compose.sim.runtime.yaml`。本机 Compose 的 `run` 会解析构建上下文，并在未显式请求构建时重建关联的 production；单独设置 `pull_policy: never` 或 `--no-deps` 未能阻止它。runtime overlay 用 `build: !reset null` 移除运行服务的构建定义，经 dry-run 和实际启动确认只使用已构建镜像。构建命令只使用 `compose.sim.yaml`，不叠加 runtime overlay。

## 单场景启动

一个入口管理 world、实体生成与 `/clock`：

```bash
docker compose -f docker/compose.sim.yaml -f docker/compose.sim.runtime.yaml run --rm simulation \
  ros2 launch greenhouse_nav2_bringup simulation_matrix.launch.py \
  world:=maize_field robot:=husky_a200 headless:=true output:=/evidence/single_husky
```

参数：`world`（maize_field / orchard / pipeline）、`robot`（jackal_j100 / husky_a200 / x500）、`headless`、`seed`（玉米田生成 seed，构建期固定为 20261006）、`task`、`output`。地面车出生点按场地设置（玉米田地头角、果园集结区、丘陵基站平地）；X500 出生点与航线见 `src/bringup/greenhouse_sim_air/config/x500_sites.yaml`，航点高度按场地净空预先核验。

导航链路与仿真车共用入口：默认接入 `closed_loop.launch.py` 的 FAST-LIO、地形、FAR、MPPI 与门控；`controller:=cmu` 仅用于独立对照。门控读取 `/cmd_vel/nav` 并独占 `/cmd_vel`；命令经 `greenhouse_sim_cmd_sink` 转 `/platform/cmd_vel`，由 `gz_ros2_control` 执行。雷达无回波中的非有限坐标经过仿真过滤器剔除，有效点的字段、字节、时间戳与 frame 保留。FAST-LIO 发布 odom→base_link，机器人描述发布传感器与车轮 TF；关闭平台控制器的重复 odom TF。接触传感器仅附加到既有碰撞体，不改变几何。

地面导航在真实 IMU、接触与单一时钟证明车辆已落稳后启动：加速度模长距 9.81 m/s² 小于 0.5、角速度小于 0.1 rad/s，持续一个模拟秒，同时要求消息新鲜。出生下落期间的零比力曾使 FAST-LIO 的重力归一化产生巨值；同场地延后到落稳再启动后，实测定位恢复为厘米级静止波动。此等待只影响初始化，不修改传感器数据，也不放宽导航未知区、footprint 或安全门控条件。120 墙钟秒内未落稳则拒绝启动导航，并记录阻塞。

IMU、LiDAR 与 RGB-D 现在使用独立桥接进程，相机桥仅在 ROS 存在订阅者时转发。果园的对照实验中，共用相机桥的 IMU 时间戳落后单一时钟最多 3.81 s；停止相机转发而保留同一物理传感器后，延迟范围为 -3 至 12 ms。另一次直接读取 Gazebo IMU 与时钟证明原始传感器新鲜，ROS IMU 却积压约 9.7 s。隔离修复后，同时订阅相机图像和 7.37 MB 点云，收到 49 张图像、52 帧相机点云及 749 帧 LiDAR，IMU 延迟仍在 -12 至 15 ms，退出无遗留进程（[实测记录](media/simulation/evidence/orchard-isolated-bridge-diagnostic.json)）。修复隔离传输工作，不重写时间戳或放宽新鲜度阈值。相机仍保留原始更新率、图像和点云接口；`lazy` 配置来自 [ros_gz_bridge Jazzy 的解析实现](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/bridge_config.cpp)。

在 WSL2 上使用已经验证的 D3D12 GPU/EGL 配置：

```bash
docker compose -f docker/compose.sim.yaml -f docker/compose.sim.runtime.yaml -f docker/compose.sim.wsl.yaml run --rm simulation \
  ros2 launch greenhouse_nav2_bringup simulation_matrix.launch.py \
  world:=maize_field robot:=x500 headless:=true output:=/evidence/single_x500
```

此配置映射 `/dev/dxg` 与 WSL GPU 库，默认选择 NVIDIA；其他适配器可设置 `SIM_GPU_ADAPTER`。Linux 原生 GPU 需要按本机驱动映射 `/dev/dri`。软件渲染可能低于导航消息新鲜度要求；应记录阻塞或失败，不放宽安全阈值。仿真容器默认限制为 6 GiB 内存、8 GiB 总内存与交换空间，8 个 CPU；通过 `SIM_CPUS` 调整。结果记录实际 cgroup CPU/内存配额及渲染适配器，并将这些设置纳入恢复哈希。

## 九组合批量测试

```bash
docker compose -f docker/compose.sim.yaml -f docker/compose.sim.runtime.yaml -f docker/compose.sim.wsl.yaml run --rm simulation \
  ros2 run greenhouse_nav2_bringup run_matrix.py --output /evidence/matrix --runs 3
```

或使用安装入口 `ros2 run greenhouse_nav2_bringup run_matrix.py --output ... --runs 3`。矩阵为 3 场地 × 3 机器人，每组合默认三次独立重置；每次运行记录配置、seed、定位来源与结果，输出 `matrix_results.json` / `matrix_results.csv` 与单次运行 JSON、日志。地面组合记录任务结果、目标误差、路径长度、耗时与任务状态；空中组合记录航点与高度误差、起降结果。结果区分通过、失败、阻塞与未运行；修复集成问题后重跑，不通过删障碍或放宽阈值掩盖失败。

中断后追加 `--resume`，继续同一输出目录中缺少的运行。已完成记录保持不变；实现或配置哈希改变时拒绝复用，应选择新目录。单次中断重试的 runtime 目录带 `_attemptN`，不会覆盖旧日志与 PX4 rootfs。可用 `--only orchard:husky_a200` 选择一个已配置组合。容器运行时可传入 `SIM_IMAGE_ID`（`docker image inspect "$SIM_IMAGE" --format '{{.Id}}'` 的结果），将镜像身份纳入记录和恢复校验。

每轮退出记录实际进程清理：Gazebo 的 Ruby 包装器可先退出，而另一个进程组中的服务器仍存活。运行器先捕获进程血缘及本轮精确 world 文件参数，再按 `/proc` 启动时间核实身份，清理仍存活的本轮进程。记录 `process_cleanup` 的追踪、强制终止和遗留 PID；若有遗留，停止矩阵，不启动下一世界。只等 launch 退出不能证明独立重置。本轮实际复现过遗留服务器导致容器 OOM，日志与回归验证见 [诊断](media/simulation/evidence/process-cleanup-diagnosis.md)。

Clearpath controller spawner 的服务响应和状态切换等待也设为 120 墙钟秒；默认五秒的切换等待曾在果园资源加载期间失败。此项仅允许启动完成，不改变物理落稳、导航数据新鲜度或控制安全阈值。

地面任务配置为 [ground_sites.yaml](../src/bringup/greenhouse_nav2_bringup/config/simulation/tasks/ground_sites.yaml) 的正常、地形挑战和危险区三个用例。普通控制器错误不计为正确拒绝；危险目标报告 `FAR unreachable` 还须同一运行中的正常用例已成功，才具备拒绝验收的前提。记录接触传感器测得的碰撞，排除正常轮地接触；同时记录连续命令下的卡滞、roll/pitch、倾覆、目标误差、路径长度及模拟时间/墙钟时间之比。超时与数据未就绪保留已测指标，未执行用例明确标记。

X500 用 PX4 真实消息执行全流程，自动发送本地 MAVLink GCS 心跳，保留正常解锁检查。所有航段和返回采用场地固定巡航高度；先垂直爬升，再水平巡检，回到起降区后下降。记录连续位置轨迹、ULog、悬停误差与速度、航点误差、巡航失高、roll/pitch、起降和模式状态；位置/姿态/时钟失效或飞行中退出 Offboard 明确失败。悬停要求位置及速度稳定持续三个模拟秒。

返航继续保持 Offboard 的 home setpoint，直到水平误差小于 0.3 m、速度小于 0.3 m/s 且高度合格，持续两个模拟秒，再切换 AUTO_LAND。直接在 1.5 m 航点半径边界切降落曾使高速返航的惯性越出起降柱；该修复收紧切换条件，仍连续执行原有起降柱和高度净空检查。失败结果记录各遥测话题的墙钟接收年龄。

定位来源：地面组合使用模拟传感器驱动的 FAST-LIO 定位（`localization: simulation_sensors`）；以 ground truth 辅助导航的运行标记为 navigation-only，不计入端到端验收。

## 自动截图

截图使用世界内第三人称相机（`worlds/simulation_camera.sdf`，1920×1080，1 Hz 静态截图用途）经 ros_gz 桥接与保存器采集；Gazebo 以 OGRE2/EGL 离屏渲染。此相机的低帧率减少额外渲染与大图传输开销，不改变机器人的雷达、IMU、RGB-D 或 PX4 传感器频率。RViz 截图单独验证显示环境。

```bash
# 九组合首轮自动采集：9 张任务图、3 张全景、3 张真实 RViz 图。
docker compose -f docker/compose.sim.yaml -f docker/compose.sim.runtime.yaml -f docker/compose.sim.wsl.yaml run --rm simulation \
  ros2 run greenhouse_nav2_bringup run_matrix.py --output /evidence/matrix --runs 3 \
  --capture-first-run --media-output /media

# 单次任务及截图，输出目录或运行编号须未被已完成记录占用。
docker compose -f docker/compose.sim.yaml -f docker/compose.sim.runtime.yaml -f docker/compose.sim.wsl.yaml run --rm simulation \
  ros2 run greenhouse_nav2_bringup run_simulation_run.py maize_field jackal_j100 \
  --run-id maize_field_jackal_j100_single --output /evidence/single \
  --capture --capture-rviz --capture-panorama --media-output /media
```

每张图保存运行编号、world/robot、相机位姿、仿真时间与测试状态（JSON sidecar）；逐张排除黑图、空图、缺模型与遮挡。正式图片放 `docs/media/simulation/`，README 选用三张代表性主图。

任务相机使用入口写出的实际 `camera.json`，等地面任务被接受或 X500 已起飞后采集。全景只移动独立的 `sim_camera`，不改机器人 pose。文件名含运行编号及 `_task`、`_panorama` 或 `_rviz`；PNG 保留原始分辨率，JPG 用于 GitHub。RViz 使用独立 Xvfb 1920×1080 屏幕和软件 GLX，订阅真实定位点云、地形及机器人描述。首条具有至少两个节点的真实 `/planning/global_path` 原样保存在专用显示快照话题，供 RViz 渲染；不改坐标、不参与控制。图中蓝色路径是历史规划，可能已被取消，sidecar 保存原始路径时间与坐标及实际截图时的任务状态。控制仍读取原话题并执行取消与过期规则。显示接收后另等待两秒，避免把话题到达误认为画面已绘制。RViz 与 Gazebo GPU 渲染分别验证，检测黑图和话题可用性不能替代逐张视觉复核，sidecar 保留复核状态。

## 几何与能力边界

镜像构建执行 `tools/inspect-simulation-geometry.py`，产出 `/opt/greenhouse_sim/geometry.json`。读取实际碰撞 mesh、所有 1015 株玉米模型及 heightmap，应用模型/mesh 的坐标变换；另读取 X500、脚架和相机的实际碰撞 box，核验初始出生位置不穿入地形。X500 入口要求该报告通过，且航线配置 SHA 与报告一致；改航线后须重建。净空覆盖起降垂直柱、所有航段及返回，使用全世界最高几何的保守上界，包含机体半径、位置误差与余量。

| 场地 | 地形 world Z 范围（m） | 坡度最大值 / 95 分位（度） | 固定巡航高度（m，相对 home） | 保守垂直净空（m） |
| --- | --- | --- | --- | --- |
| maize_field | 0.00331–0.40000 | 66.712 / 9.730（像素梯度） | 3 | 1.992 |
| orchard | -0.10000–0.07080 | 17.985 / 8.802（碰撞三角面） | 6 | 3.912 |
| pipeline | -3.71875–6.72565 | 89.547 / 54.503（含陡壁） | 10 | 3.092 |

这是静态资产的几何核验，不是车辆可通行坡度或实机性能。玉米 heightmap 按 Harmonic DART/Bullet 的 `pixel / maximum_pixel * size.z` 缩放并处理 sampling 与行方向，不用名义 `pixel/255` 高程。玉米视觉最高 world Z 约 1.058 m，果园交付碰撞几何最高约 1.988 m；不套用其他树种或模型的名义高度。没有涉水、沉陷及柔性作物物理模型，不宣称对应测试通过。

丘陵 X500 的原出生区足下高差为 0.195 m，实际侧倾导致 PX4 拒绝解锁；改用 `(-3, 3, 0.1)` 的实测平地，足下高差 0.030 m。保持原始碰撞、惯性和检查阈值，重新核验整条 10 m 高度航线与起降区后，真实全流程飞行通过。静态净空通过本身不保证地面姿态稳定，详见 [诊断与实证](media/simulation/evidence/pipeline-air-spawn-diagnosis.md)。

## 2026-10-07 实测结果

候选镜像 `sha256:dea19f601f3e14ace38fd3da54b20d3b1104ed70fc37b4e29be4e7f3caa9c09d` 的九组合共 27 次独立重置已执行：8 次通过、19 次失败，无阻塞或未运行。18 次地面任务全部失败；空中玉米田、果园各 3 次通过，丘陵 2 次通过、1 次起飞越出已核验的 home 柱而失败。该失败保留，未放宽净空或姿态阈值。每次清理的 `remaining_pids` 均为空，任务容器的 cgroup OOM 与 OOM kill 均为零。

全部运行使用同一镜像、实现 SHA256、seed 20261006 和资源配额（8 CPU、6 GiB 内存）；29 项安装的 SIM 源码、配置及几何工具与当前源码一致。固定参数的玉米田两次独立生成六项产物一致，共 1015 株（509 株 maize_01、506 株 maize_02）；未指定 preset，CLI 构造参数实际生效，杂草和垃圾关闭。PX4 与 Agent 的新构建已完成；三项 ROS 命令序列化/点云字节回归、两项真实进程清理回归以及一项 PX4 实际消息启动回归通过。不同检查的镜像、输入和适用范围见 [构建与验证证据](media/simulation/evidence/README.md)。

交付 15 张逐张视觉复核的 1920×1080 真图（3 全景、9 任务、3 RViz），同时保留 PNG、GitHub 用 JPG 与 JSON 元数据。丘陵首轮定位过期，未获得可渲染 FAR 路径；使用同一镜像、同一实现额外独立运行补拍 RViz，不替换原失败记录，不加入 27 次矩阵统计。见 [相册](media/simulation/README.md)、[JSON](media/simulation/evidence/candidate-matrix/matrix_results.json)、[CSV](media/simulation/evidence/candidate-matrix/matrix_results.csv) 与 [证据核对清单](media/simulation/evidence/matrix-validation.json)。

地面链已运行真实模拟传感器定位、地形/FAR、MPPI、适配器、门控与底盘接口，实际正常、挑战及危险用例均失败，不能称为成功导航或正确危险区拒绝。部分丘陵运行还报告 `global odometry unavailable or expired`。完整矩阵日志与对应运行目录保留全部任务状态、接触/姿态指标与轨迹；后续工作应处理近域观测覆盖与数据新鲜度，不应把零速安全停机作为通行成功。

已有诊断发现 VLP16 近场未观察地形会使 MPPI 拒绝轨迹；该失败不以“未知区置空闲”、缩 footprint 或放宽新鲜度掩盖。早期空中全流程诊断使用旧出生点/高度，只作为排错证据，不计入最终矩阵。最终结果以对应候选镜像的 JSON/CSV、日志和图片为准。

FAR 上游仅在障碍点非空时初始化图，导致已观测、没有障碍的地头永久 not-ready。本轮实际测得 1695 个空闲地形点、零障碍点，补丁 0005 允许非空实测空闲地形初始化；修复后 FAR 输出真实完整路径，MPPI 随后明确报告 `Optimizer fail to compute path`。诊断网格中车身附近六格全部未知，2 m 方形近域 100 格中 96 格未知，MPPI、适配器、门控和底盘接口实测均输出零速。这说明导航任务失败并安全停止，不能称为完成正常通行或正确识别危险区域。

最终候选在已成功构建的 production/SIM 镜像上增量安装重新编译的 FAR 单一可执行文件，并安装修复后的 SIM 运行脚本。全部 29 个 FAR 编译输入与固定源码及补丁一致，编译 34.7 s 成功，二进制 SHA256 为 `d42fe736b103ff5c82c8b684e0f83c08cbcbc81052f38d6601de9424c2d06191`；SIM 安装文件与当前源码哈希逐项核对。四步完整构建会自动应用 0005，但补丁加入后未重复整套 production 构建，不能把增量验证写为最后版本的全量重建。相关证据保存在 [evidence](media/simulation/evidence/)。

矩阵的非截图轮次省略可选 `capture_marker` 参数，采用任务节点的空字符串默认值。曾把空值直接拼成 `capture_marker:=`，导致 ROS 参数解析失败而未飞行；失败日志保留，修复后重新执行，不计为飞行性能失败。

每次空中重置还先等待当轮实际 `VehicleLocalPosition` 消息，不能只查询话题名称。旧矩阵中一轮已发现话题，但 30 墙钟秒内没有任何 PX4 遥测，仿真仅推进 2.41 s，任务因此过早超时。运行器现在使用原有 180 s 启动等待来接收真实有限位置消息，再启动任务；正常解锁检查、任务的 30 s 就绪等待和 2 s 遥测新鲜度要求保持原值。独立 ROS domain 179 的接口回归证明只有已发现话题时不会提前放行，实际消息到达后才继续（[回归](media/simulation/evidence/px4-live-telemetry-startup-regression.log)）；该合成接口测试不计为飞行证据。修复后无截图实飞完整通过，并重新执行同一候选版本的正式矩阵。

## 环境注意

- 容器内有多个 docker 桥接口时，gz-transport 的自动选路可能把服务发现绑到错误接口；矩阵入口统一设置 `GZ_IP=127.0.0.1`。
- 虚拟玉米田的上游 `sincurved` 段生成器在部分行数/缺株参数下不终止；固定参数改用 `straight`+`curved` 混合行，保留直行与曲行形态。
- PX4 v1.17 要求 Micro-XRCE-DDS-Agent v2.4.3（v3.x 不兼容），版本在 `third_party/px4.repos` 固定。
