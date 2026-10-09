# 构建与运行

命令从仓库根目录执行，需要 Linux 或 WSL。`src/` 同时包含 catkin 和 ament 包，按以下入口选择独立工作区。

## ROS 2 Jazzy 导航

需要 Docker BuildKit 和支持 `additional_contexts` 的 Compose 2.17+。当前交付平台为 amd64。

```bash
bash tools/build-ros2.sh
```

源码版本和补丁位于 [third_party](../third_party/README.md)，基础镜像 digest 固定在 `docker/Dockerfile`。`docker/config/{build,runtime,simulation}.packages` 按用途列出 APT 包名，安装时解析软件源当前版本。首次构建需要访问镜像、Git 和软件包源。`BUILD_JOBS` 控制编译并行度，默认 2。构建无需部署目录或底盘设备配置。

容器配置、仿真和启动命令见 [容器说明](../docker/README.md)，任务操作见 [导航说明](../src/navigation/greenhouse_mppi_navigation/README.md)。

## ROS 1 Noetic SLAM

需要 ROS Noetic、PCL、Eigen、OpenMP、GTSAM 4.2、Livox-SDK2 及各包 `package.xml` 中的依赖。自定义依赖前缀通过 `CMAKE_PREFIX_PATH` 提供。固定源码可按需准备：

```bash
python3 tools/prepare-dependencies.py ros1 .tmp/slam-deps gtsam livox_sdk2
```

脚本需要 Git 和 PyYAML，目标目录须尚不存在。按两个依赖各自的 CMake 安装入口完成安装，再构建本库三个 ROS 1 包：

```bash
bash tools/build-ros1.sh .tmp/ros1-slam
source .tmp/ros1-slam/devel/setup.bash
roslaunch fast_lio_super mapping_mid360.launch map_backend:=octvox loop_enabled:=true rviz:=false
```

脚本链接本库源码，目标工作区须尚不存在。后续在生成工作区执行 `catkin_make -j2 -DROS_EDITION=ROS1`。传感器配置按实际标定填写。

## FAST-LIVO2

需要 Noetic、GCC 9、Eigen、OpenCV、PCL 和包中列明的 ROS 依赖。以下入口准备固定 Sophus/vikit，并直接编译本库 `fast_livo`：

```bash
bash tools/build-fastlivo2.sh .tmp/fastlivo-build
source .tmp/fastlivo-build/devel/setup.bash
roslaunch fast_livo mapping_avia.launch
```

运行前配置传感器、相机与内存预算，详见 [FAST-LIVO2](../src/localization/fast_livo/README.md)。工作区支持增量构建。

## 开发检查

测试随所属 ROS 包保存。Jazzy 包构建时启用 `-DBUILD_TESTING=ON`，随后执行对应工作区的 `colcon test` 与 `colcon test-result --verbose`。地图与回环独立检查入口为 `src/localization/fast_lio_super/test/CMakeLists.txt`。

FAST-LIVO2 生命周期检查：

```bash
FASTLIVO_MEMORY_SANITIZE=ON bash tools/build-fastlivo2.sh .tmp/fastlivo-build check
```

依赖准备、临时构建和测试日志统一放在 `.tmp/`，使用完成后清理。长期开发可显式指定自己的构建工作区。算法运行输出遵循包内约定，例如 FAST-LIVO2 的 `Log/`；这些生成内容通过 Git ignore 排除。
