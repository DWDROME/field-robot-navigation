// SPDX-License-Identifier: BSD-2-Clause
#include "slam/map/ikd_tree_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace slam::map {

// KD_TREE 的 PointVector 恰好是 slam::PointVec 的别名，两者可直接互换。
static_assert(std::is_same<typename KD_TREE<PointT>::PointVector, PointVec>::value,
              "KD_TREE<PointT>::PointVector must match slam::PointVec");

namespace {
inline uint64_t nowNs() {
  using namespace std::chrono;
  return static_cast<uint64_t>(
      duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}
inline BoxPointType toBoxPoint(const BoundingBox& b) {
  BoxPointType out;
  out.vertex_min[0] = b.min.x();
  out.vertex_min[1] = b.min.y();
  out.vertex_min[2] = b.min.z();
  out.vertex_max[0] = b.max.x();
  out.vertex_max[1] = b.max.y();
  out.vertex_max[2] = b.max.z();
  return out;
}
}  // namespace

IkdTreeMap::IkdTreeMap()
    : tree_(0.5f, 0.6f, 0.2f /* delete_param, balance_param, downsample size；与 FAST-LIO2 默认一致 */) {}

IkdTreeMap::~IkdTreeMap() = default;

void IkdTreeMap::configure(const MapConfig& cfg) {
  cfg_ = cfg;
  if (cfg.voxel_size > 0.f) tree_.set_downsample_param(cfg.voxel_size);
}

void IkdTreeMap::insert(const PointVec& world_pts) {
  if (world_pts.empty()) return;
  const uint64_t t0 = nowNs();

  // KD_TREE::Add_Points / Build 均以非 const 引用接收；拷贝一次以匹配签名。
  PointVec buf = world_pts;
  if (tree_.Root_Node == nullptr) {
    tree_.Build(buf);
  } else {
    tree_.Add_Points(buf, /*downsample_on=*/true);
  }

  last_insert_ns_.store(nowNs() - t0, std::memory_order_relaxed);
}

void IkdTreeMap::insertNoDownsample(const PointVec& world_pts) {
  if (world_pts.empty()) return;
  const uint64_t t0 = nowNs();
  PointVec buf = world_pts;
  if (tree_.Root_Node == nullptr) {
    tree_.Build(buf);
  } else {
    tree_.Add_Points(buf, /*downsample_on=*/false);
  }
  last_insert_ns_.store(nowNs() - t0, std::memory_order_relaxed);
}

int IkdTreeMap::knn(const nn::Query& q, std::array<Neighbor, 8>& out) const {
  // B-1.c：热路径零锁。ikd-Tree::Nearest_Search 自身线程安全（支持并发读）；
  // 只用 relaxed atomic 记录最近一次 query 的耗时，OMP 并行下允许 race，
  // 值仅作为 stats 监控、不影响正确性。
  const uint64_t t0 = nowNs();
  const int k = std::min(q.k, 8);

  PointVec            nearest;
  std::vector<float>  dists;
  nearest.reserve(k);
  dists.reserve(k);

  tree_.Nearest_Search(q.center, k, nearest, dists,
                       std::isfinite(q.max_dist) ? q.max_dist : INFINITY);

  const int n = static_cast<int>(std::min<std::size_t>(nearest.size(), 8));
  for (int i = 0; i < n; ++i) {
    out[i].p        = nearest[i];
    out[i].dist2    = (i < static_cast<int>(dists.size())) ? dists[i] : 0.f;
    out[i].voxel_id = 0;
  }

  last_query_ns_.store(nowNs() - t0, std::memory_order_relaxed);
  return n;
}

bool IkdTreeMap::fitPlane(const nn::Query& q, PlaneCoef& out) const {
  std::array<Neighbor, 8> nbr{};
  const int n = knn(q, nbr);
  if (n < 3) { out.valid = false; return false; }

  Eigen::MatrixXf A(n, 3);
  Eigen::VectorXf b = -Eigen::VectorXf::Ones(n);
  for (int i = 0; i < n; ++i) {
    A(i, 0) = nbr[i].p.x;
    A(i, 1) = nbr[i].p.y;
    A(i, 2) = nbr[i].p.z;
  }
  Eigen::Vector3f nvec = A.colPivHouseholderQr().solve(b);
  const float norm = nvec.norm();
  if (!std::isfinite(norm) || norm < 1e-6f) { out.valid = false; return false; }
  nvec /= norm;
  const float d = 1.0f / norm;

  float max_res = 0.f;
  for (int i = 0; i < n; ++i) {
    const float r = std::fabs(nvec.dot(Eigen::Vector3f(nbr[i].p.x, nbr[i].p.y, nbr[i].p.z)) + d);
    max_res = std::max(max_res, r);
  }
  out.abcd      = Eigen::Vector4f(nvec.x(), nvec.y(), nvec.z(), d);
  out.curvature = max_res;
  out.valid     = (max_res < 0.1f);
  return out.valid;
}

CloudPtr IkdTreeMap::snapshot(float voxel) const {
  (void)voxel;
  PointVec all;
  tree_.flatten(tree_.Root_Node, all, NOT_RECORD);
  CloudPtr cloud(new CloudT());
  cloud->reserve(all.size());
  for (auto& p : all) cloud->push_back(p);
  return cloud;
}

void IkdTreeMap::flattenInto(PointVec& out) const {
  tree_.flatten(tree_.Root_Node, out, NOT_RECORD);
}

int IkdTreeMap::removeBoxes(const std::vector<BoundingBox>& boxes) {
  if (boxes.empty()) return 0;
  std::vector<BoxPointType> bp;
  bp.reserve(boxes.size());
  for (const auto& b : boxes) bp.push_back(toBoxPoint(b));
  return tree_.Delete_Point_Boxes(bp);
}

void IkdTreeMap::cropAround(const Eigen::Vector3d& c, float radius) {
  (void)c; (void)radius;  // Phase A 不启用：FAST-LIO2 FoV 裁剪已由 removeBoxes 覆盖。
}

void IkdTreeMap::drainRemoved(PointVec& out) {
  tree_.acquire_removed_points(out);
}

bool        IkdTreeMap::empty()     const { return tree_.Root_Node == nullptr; }
std::size_t IkdTreeMap::size()      const { return static_cast<std::size_t>(tree_.size()); }
std::size_t IkdTreeMap::validSize() const { return static_cast<std::size_t>(tree_.validnum()); }

MapStats IkdTreeMap::stats() const {
  MapStats s;
  s.num_points     = validSize();
  s.num_voxels     = 0;  // ikd-tree 不按 voxel 计数
  s.bytes          = 0;
  s.last_insert_ms = last_insert_ns_.load(std::memory_order_relaxed) * 1e-6;
  s.last_query_ms  = last_query_ns_.load(std::memory_order_relaxed) * 1e-6;
  return s;
}

std::unique_ptr<IMapManager> makeIkdTreeMap() { return std::make_unique<IkdTreeMap>(); }

}  // namespace slam::map
