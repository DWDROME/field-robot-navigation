# FAST-LIO Super 的 Jazzy 入口

本 ament 包编译相邻 `fast_lio_super` 和 `slam_octvox` 的共享算法，导出 `fast_lio_super_ros2::fastlio_map` 和 `fastlio_graph`。锁定 ROS 2 FAST-LIO 通过 `third_party/patches/fast_lio/0002-pluggable-map-backend.patch` 使用地图库。

## 构建与启动

完整构建见 [构建说明](../../../docs/build.md)。已配置 Jazzy、PCL 与 GTSAM 4.2 的环境可独立构建：

```bash
colcon build --base-paths src/localization/fast_lio_super_ros2 --cmake-args -DEigen3_DIR=/usr/share/eigen3/cmake -DGTSAM_DIR=/your/gtsam/lib/cmake/GTSAM
source install/setup.bash
ros2 launch fast_lio_super_ros2 backend.launch.py
```

`backend.launch.py` 为算法独立入口；完整导航装配见 [greenhouse_mppi_navigation](../../navigation/greenhouse_mppi_navigation/README.md)。输入为时间完全一致的 odometry 与注册点云。全局校正拥有 `map → odom`，局部 odometry 连续；启用回环时关闭 identity 校正发布者。

参数、frame、topic、QoS 和资源预算见 [闭环合同](../fast_lio_super/docs/CLOSED_LOOP.md)。`test/check_backend_interfaces.py` 提供合成点云与里程计的节点接口检查。
