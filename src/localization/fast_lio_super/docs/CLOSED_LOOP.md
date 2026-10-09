# FAST-LIO Super 闭环实现

共享算法位于 `include/slam/graph` 和 `src/slam/graph`，不依赖 ROS。ROS 1 的 `backend_ros1.cpp` 和 ROS 2 的 `fast_lio_super_ros2/src/backend_node.cpp` 负责消息同步、坐标转换和输出。两者都使用消息时间完全一致的原始里程计与世界系注册点云，将点云转换到关键帧自身坐标系。

## 算法与资源

累计平移或旋转角触发关键帧。Scan Context 按时间和帧 ID 排除近邻，通过 ring key 筛选有界候选，再比较循环 sector 移位提供航向初值。候选子地图也排除近时/相邻帧。PCL GeneralizedIterativeClosestPoint 使用候选初始变换，经过收敛、有限位姿、匹配数量、匹配比例和均方误差门限后才产生约束。

GTSAM/iSAM2 接收首帧先验、由原始相邻位姿计算的里程计因子以及验证通过的回环因子。回环因子使用 Cauchy 鲁棒核。优化结果不会写回 IESKF。校正定义为 `T_map_odom = T_map_keyframe_optimized * inverse(T_odom_keyframe_original)`，局部里程计持续保持连续。

后台只有一个工作线程。输入队列丢弃最旧帧，点数在入队前有界；关键帧数量、单帧点数、总点数分别受配置限制。达到历史预算后停止新增关键帧并发布诊断，保留图中历史引用。结束时清空待处理队列、唤醒并 join。关闭回环的 launch 不创建 Backend。

参数默认值见 `include/slam/graph/backend.hpp`；ROS 1 示例为 `config/loop_backend.yaml`，ROS 2 示例为 `fast_lio_super_ros2/config/backend.yaml`。阈值是可配置的软件默认值，未声称适用于未经标定的实车。

## 启动和输出

ROS 1：

```bash
roslaunch fast_lio_super mapping_mid360.launch map_backend:=octvox loop_enabled:=true rviz:=false
```

前端保留 `/Odometry`、`/cloud_registered` 和 `camera_init`/`body`。回环额外发布 `/slam/global_odometry`、`/slam/corrected_path`、`/slam/corrected_map`、`/diagnostics` 和 `map → camera_init`。路径和校正地图只在有订阅者时按 `output_period` 生成。

ROS 2：

```bash
ros2 launch greenhouse_nav2_bringup closed_loop.launch.py deployment:=/absolute/deployment.yaml map_backend:=octvox
```

前端独占 `odom → base_link`；启用回环时 Backend 独占 `map → odom`，关闭时只创建 identity 发布者。部署外参定义 `T_imu_lidar` 和 `T_base_lidar`，前端以 `T_imu_body = T_imu_lidar * inverse(T_base_lidar)` 转换输出位姿、点云和连续 body twist，避免将 IMU 坐标直接标记成 `base_link`。静态节点独占 `base_link → livox_frame`。

`/localization/odometry` 和 `/localization/registered_cloud_odom` 是原始连续局部输出；`/localization/global_odometry`、`global_pose`、`corrected_path` 和 `corrected_map` 使用 map frame。点云同步使用 sensor-data QoS；校正输出和诊断使用 reliable/volatile。时间来自匹配帧，TF 与当前原始里程计同一时间。校正地图生成节奏不影响前端地图查询。

## 构建来源

ROS 1 保持 catkin；ROS 2 单独构建 `fast_lio_super_ros2` 和锁定 FAST-LIO ROS 2 前端。`third_party/ros2.repos` 与 `third_party/common.repos` 固定来源。`0002-pluggable-map-backend.patch` 接入共享地图库和部署 frame 转换，不复制第三方研究参考工程。GICP 使用 PCL，优化使用 GTSAM 4.2；容器从固定 GTSAM commit 构建。

定向算法检查：

```bash
cmake -S src/localization/fast_lio_super/test -B build/fastlio-contract -DGTSAM_DIR=/your/gtsam/lib/cmake/GTSAM
cmake --build build/fastlio-contract -j2
ctest --test-dir build/fastlio-contract --output-on-failure
```

ROS Jazzy 的 GTSAM Debian 包将内部 Metis 库放在 multiarch lib 目录，`fast_lio_super_ros2` 的环境 hook 将其加入 `LD_LIBRARY_PATH`。构建共享库时确保 Eigen 与 GTSAM 的构建版本一致。本文描述实现；编译、接口、回放、性能和实机结果分别记录，不能由源码推导实机成绩。
