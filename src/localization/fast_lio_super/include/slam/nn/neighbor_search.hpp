// SPDX-License-Identifier: BSD-2-Clause
// INeighborSearch：IESKF 侧构造残差时的唯一邻居查询接口。
// 目的是让 ikd-tree / HKNN / 其他后端在 IESKF 代码不变的前提下互换。
#pragma once

#include <array>
#include <vector>

#include "slam/common_types.hpp"

namespace slam::nn {

struct Query {
  PointT  center{};
  int     k        = kDefaultKnn;
  float   max_dist = 1.0f;   // 米；硬上限，正无穷表示不限距离
};

class INeighborSearch {
 public:
  virtual ~INeighborSearch() = default;

  // 返回真实邻居数（<= k）。结果按 dist2 升序。
  // 小固定长度缓冲（<=8），避免每帧 heap 分配。
  virtual int knn(const Query& q,
                  std::array<Neighbor, 8>& out) const = 0;

  // 平面拟合便捷入口，供 FAST-LIO2 style residual 直接消费。
  // 默认实现：调用 knn 后在基类外部做拟合即可；子类可 override 做更优实现。
  virtual bool fitPlane(const Query& q, PlaneCoef& out) const = 0;

  virtual const char* name() const = 0;   // "ikdtree" / "hknn" ...
};

}  // namespace slam::nn
