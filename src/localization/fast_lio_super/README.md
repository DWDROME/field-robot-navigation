# FAST-LIO Super

基于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 的地图与闭环扩展。提供统一地图接口、ikd-Tree / OctVox 后端，以及 Scan Context → GICP → GTSAM/iSAM2 闭环。前端通过地图接口查询、插入和裁剪；回环后台输出全局校正，局部里程计保持连续。

## 构建与启动

ROS 1 包名为 `fast_lio_super`，独立 catkin 构建见 [构建说明](../../../docs/build.md)。

```bash
roslaunch fast_lio_super mapping_mid360.launch map_backend:=octvox loop_enabled:=true rviz:=false
```

`map_backend` 可选 `ikdtree` 或 `octvox`，`loop_enabled:=false` 关闭回环。传感器、噪声、外参与地图预算位于 `config/`，使用前按设备填写。

ROS 2 入口由相邻 [fast_lio_super_ros2](../fast_lio_super_ros2/README.md) 提供，共享 `include/slam` 与 `src/slam`。

## 接口

- [SLAM_CORE_API](docs/SLAM_CORE_API.md)：地图插入、KNN、删除、空间裁剪、快照与统计。
- [CLOSED_LOOP](docs/CLOSED_LOOP.md)：关键帧、图优化、资源预算、话题与 TF。
- `test/`：地图和回环合同检查源码。

## 来源

FAST-LIO2 与 ikd-Tree 来自 HKU MARS；OctVox 来自 [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)，实现在相邻 `slam_octvox` 包中。上游示例媒体和论文见原仓库。原始 LICENSE 与源文件版权声明保留。
