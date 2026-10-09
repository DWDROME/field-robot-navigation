# slam_octvox

Header-only catkin 包，为 `fast_lio_super` 的统一地图接口提供 OctVox 数据结构、HKNN 查表和 robin_map 容器。

`include/slam_octvox` 来自 [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO) 的 ROS 1 分支，保留原 GPL 许可证。`include/tsl` 使用文件头所载 MIT 许可。项目维护的 `OctVoxMap.hpp` 配合地图适配器提供容量淘汰、空间删除与索引维护，地图语义见 [SLAM_CORE_API](../fast_lio_super/docs/SLAM_CORE_API.md)。

## 使用

catkin 消费方在 manifest 声明 `slam_octvox` 依赖，并通过 `catkin_INCLUDE_DIRS` 包含头文件：

```cpp
#include "slam_octvox/OctVoxMap.hpp"
using Map = LI2Sup::OctVoxMap<Eigen::Vector3f, float>;
```

ROS 2 的 `fast_lio_super_ros2` 直接引用相邻目录的 `include/`。并行查询和串行地图维护由消费方调度。
