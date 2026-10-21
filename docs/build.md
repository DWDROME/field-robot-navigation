# 构建

main 使用 Ubuntu 20.04 与 ROS 1 Noetic，所有 ROS 包在 src 下，保持原包名与 catkin 构建方式。ROS 2 升级与 Gazebo 构建请先切换到 upgrade/ros2，再读取该分支的本文档。

## ROS 1 工作区

准备 Noetic、PCL、Eigen、OpenMP、OMPL、OctoMap、串口依赖和各 package.xml 所列依赖。局部规划和全局定位还需要包内说明列出的 CasADi、Open3D、ros_numpy 等 Python 依赖。Livox-SDK2 独立安装，固定来源见 [third_party](../third_party/README.md)。

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash tools/build-ros1.sh
source devel/setup.bash
```

`BUILD_JOBS` 默认为 2；额外 CMake 参数可直接追加。Livox 的 package.xml 保持 ROS 1 catkin，构建参数固定 `-DROS_EDITION=ROS1`。此入口只构建 src，根目录 firmware 不参加 catkin 构建。

## 下位机

F407 使用独立 CMake/GCC/Ninja 工具链，编译与烧录命令见 [TR800](../firmware/tr800_f407/README.md)。ROS 构建不生成或烧录固件。

## 运行输入

PCD 地图和录包保留在运行数据目录，通过 `map:=/absolute/map.pcd` 提供。源码中 PCD 目录只保留建图输出位置；原有大型地图不进入 Git。雷达 JSON、串口设备、传感器外参与运动参数按实际平台配置，操作见[模式说明](modes.md)。
