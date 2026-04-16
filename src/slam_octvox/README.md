# slam_octvox

**License: GPLv3** (所有代码均继承自 Super-LIO 的 GPLv3 授权；见 `LICENSE`)

Header-only catkin 包，把 [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)
的 `OctVoxMap` + `HKNN` LUT + `tsl::robin_map` vendored 到一起，给
`fast_lio_super` 的 `slam::map::IMapManager` 做后端。

## 目录

```
slam_octvox/
├── LICENSE                         # GPLv3（Super-LIO 原件）
├── package.xml                     # license: GPL-3.0-or-later
├── CMakeLists.txt                  # header-only，只导出 include/
├── include/
│   ├── slam_octvox/
│   │   ├── OctVoxMap.hpp           # 从 Super-LIO ros1 分支 vendored，剥了 saveMap()
│   │   └── HKNN_list60_gem.h       # 从 Super-LIO ros1 分支 vendored，原样
│   └── tsl/                        # robin_map 等（MIT，见文件头）
└── docs/
```

## Vendoring notes

- 来源：`https://github.com/Liansheng-Wang/Super-LIO` `ros1` 分支
- `OctVoxMap.hpp` 做了三处**非算法**改动：
  1. 删 `#include <execution>`（原本未被使用）
  2. 删 `#include <filesystem>`
  3. 删 `saveMap()` 方法（依赖 `std::filesystem` 与 `pcl::io`；我们改走
     上层调用者用 `getMap()` 再自行落盘的路线）
- `HKNN_list60_gem.h` 未改动
- `tsl/` 全部原样保留，其 MIT 授权继续生效

## 如何被消费

消费方（如 `fast_lio_super`）在 `package.xml` 加：
```xml
<build_depend>slam_octvox</build_depend>
<exec_depend>slam_octvox</exec_depend>
```
在 `CMakeLists.txt`：
```cmake
find_package(catkin REQUIRED COMPONENTS slam_octvox ...)
include_directories(${catkin_INCLUDE_DIRS})
```
然后：
```cpp
#include "slam_octvox/OctVoxMap.hpp"
using Map = LI2Sup::OctVoxMap<Eigen::Vector3f, float>;
```

## 不做的事

- 不搬 Super-LIO 的 `super_lio.cpp`（完整前端，与 FAST-LIO2 冲突）
- 不搬 `basic` 包（V3/SE3/scalar 别名），我们直接用 `Eigen::Vector3f`
- 不引 TBB；`OctVoxMap.hpp` 本身不依赖 TBB，并发调度留给消费方（如
  `fast_lio_super` 使用 OpenMP）

## 升级

如需同步 Super-LIO 上游新版：
1. `git clone -b ros1 https://github.com/Liansheng-Wang/Super-LIO.git /tmp/sl`
2. `diff -u include/slam_octvox/OctVoxMap.hpp /tmp/sl/src/super_lio/include/OctVoxMap/OctVoxMap.hpp`
3. 合并上游改动，重复上面的三处剥离
