# TR800 履带车 ROS 1 导航

车上跑的是 Livox MID360 + FAST-LIO 这套：雷达出点云和 IMU，FAST-LIO 建图并输出里程计；有现成 PCD 地图时换成 FAST-LIO_LOCALIZATION 做全局定位；MoveBase3D 在三维点云上规划，最后 `dlrobot_robot` 把速度打成 11 字节串口帧发给 F407 下位机。

`main` 是 ROS 1 Noetic 版本，18 个包，算法代码保持原样，主要动了启动入口和文档。另外两处：

- `upgrade/ros2`：ROS 2 Jazzy 的升级版，带 Gazebo 仿真、FAR/CMU/MPPI 和独立 FAST-LIVO2。
- `firmware/tr800_f407`：下位机固件，两条分支共用同一份，单独编译烧录。

ROS 2 那套怎么跑不在这个分支，切过去看它自己的 README。

车型要说清楚：这套代码对应 **TR800 大履带**，下位机是 `TR800_F407`。仓库里也出现过 TR500 小履带，但只在固件文档的资料表里当参考，没有做引脚、参数或保护逻辑的适配，别拿 TR800 的固件直接刷 TR500。

## 装环境

Ubuntu 20.04 + ROS Noetic。除了 desktop-full，还要 PCL、Eigen、OpenMP、OMPL、OctoMap 和串口库；MoveBase3D 的局部规划用 CasADi，FAST_LIO_LOCALIZATION 要 Open3D 和 ros_numpy。

Livox-SDK2 得单独装。版本和两个 `<cstdint>` 补丁在 [third_party/README.md](third_party/README.md)，按里面的命令拉下来、打补丁再 cmake 安装。SDK 不要塞进 `src`，driver 只需要它的头文件和库。

## 编译

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash tools/build-ros1.sh          # catkin_make -DROS_EDITION=ROS1，默认 -j2
source devel/setup.bash
```

机器核心多可以 `BUILD_JOBS=8 bash tools/build-ros1.sh`。固件不在这个工作区里，catkin 不会碰它。

## 运行

建图：

```bash
bash tools/run.sh mapping hardware
```

默认存到 `src/FAST_LIO/PCD/scans.pcd`。Ctrl+C 走正常退出流程才会写文件，`kill -9` 会丢；不想存就加 `save_map:=false`。

定位和导航必须给地图，而且要绝对路径：

```bash
bash tools/run.sh localization hardware map:=/home/robot/maps/greenhouse.pcd
bash tools/run.sh navigation hardware map:=/home/robot/maps/greenhouse.pcd
```

第一次起来要在 RViz 里给初始位姿，不给定位不会收敛。目标点也用 RViz 发，多点操作看 [MoveBase3D 的 README](src/MoveBase3D/README.md)。

雷达默认读 `src/livox_ros_driver2/config/MID360_config.json`，里面的 `host_net_info` 要改成车上那块网卡的地址；临时换配置用 `user_config_path:=/abs/xxx.json`。底盘串口默认 `/dev/ttyUSB0`、115200，设备不同就加 `usart_port_name:=/dev/serial/by-id/...`。

参数都是透传给 roslaunch 的，`rviz`、`save_map`、`publish_2d_map` 这些在 [docs/modes.md](docs/modes.md) 里有完整说明。旧入口 `start_all.sh` 还留着，等价于 `navigation hardware`。

## 目录

```text
src/
  FAST_LIO/                 FAST-LIO2，建图和里程计
  FAST_LIO_LOCALIZATION/    在已有 PCD 地图里定位
  MoveBase3D/               8 个包：全局规划、GPR 地势、局部控制、rviz 工具、整车 launch
  livox_ros_driver2/        MID360 驱动，ROS1/ROS2 两套 launch
  livox_to_pointcloud2/     CustomMsg 转 PointCloud2
  pcd2pgm_package/          PCD 转 2D 栅格，给旧流程用
  dlrobot_robot/            底盘串口节点
  dlrobot/、dl_msgs/        旧底盘包和消息定义
docs/                       架构、构建、模式、数据格式、状态
tools/                      run.sh、build-ros1.sh
firmware/tr800_f407/        下位机
third_party/                Livox-SDK2 版本和补丁
```

## 固件

`firmware/tr800_f407` 用 CMake + arm-none-eabi-gcc + Ninja，preset 是 `tr800-st`，出 `dlrobot.elf/hex/bin`，DAPLink + OpenOCD 烧录，步骤都在[固件 README](firmware/tr800_f407/README.md)。

协议是固定的：11 字节命令、24 字节遥测，帧头 `0x7B`、帧尾 `0x7D`，异或校验。现在遥测第 8–19 字节是控制诊断分页，不是 IMU，别拿旧工具按加速度解。烧完可以用 `firmware/tr800_f407/tools/tr800_serial_analyzer.py --port COM4 --diag-control` 看页号和数据。

## 踩过的坑

- `localization` / `navigation` 不带 `map:=` 会直接报错退出，不是找不到节点。
- 串口打不开，先看用户在不在 `dialout` 组，再看设备名，最后才怀疑固件。
- 雷达没点云，九成是 JSON 里的 host IP 和车上网卡对不上。
- `main` 没有 ROS 1 仿真，`run.sh ... simulation` 会提示切分支，这是故意的，不是坏了。
- 遥测 8–19 是诊断字段；真的想要底盘 IMU，得先改固件配置，不是改驱动。
- 编码器第 2 路的闭环还没验证，急停和限速别省。

## 目前状态

仓库结构和启动入口整理过，也逐项对过源码，但**没在这台机器上编译，也没上车跑过**。所以“能编译、能跑”目前还只是应该。

ROS 2 那条线有一轮仿真记录，27 次里 8 次通过，地面任务基本都失败，别当成仿真可用，细节在 `upgrade/ros2` 的 docs/simulation.md。

## 许可

原包的版权头、LICENSE 和 README 声明都保留。MoveBase3D 的原 README 写明“仅供科研测试，请勿商用”，其他包按各自条款。
