// SPDX-License-Identifier: BSD-2-Clause
#include "slam/map/ikd_tree_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>
#include <limits>
#include <stdexcept>
#include <thread>
#include <pcl/filters/voxel_grid.h>

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
  // ikd-Tree uses half-open boxes; the public removal contract is closed.
  for (int i = 0; i < 3; ++i)
    out.vertex_max[i] = std::nextafter(b.max[i], INFINITY);
  return out;
}
}  // namespace

IkdTreeMap::IkdTreeMap()
    : tree_(0.5f, 0.6f, 0.2f /* delete_param, balance_param, downsample size；与 FAST-LIO2 默认一致 */) {
  tree_.set_removed_storage_enabled(false);
}

IkdTreeMap::~IkdTreeMap() = default;

void IkdTreeMap::configure(const MapConfig& cfg) {
  if (!std::isfinite(cfg.voxel_size) || cfg.voxel_size <= 0 ||
      !std::isfinite(cfg.local_crop_radius) || cfg.local_crop_radius < 0 ||
      cfg.removed_capacity == 0)
    throw std::invalid_argument("invalid ikd-Tree map configuration");
  cfg_ = cfg;
  if (cfg.voxel_size > 0.f) tree_.set_downsample_param(cfg.voxel_size);
}

void IkdTreeMap::insert(const PointVec& world_pts) {
  if (world_pts.empty()) return;
  const uint64_t t0 = nowNs();

  // KD_TREE::Add_Points / Build 均以非 const 引用接收；拷贝一次以匹配签名。
  PointVec buf = world_pts;
  buf.erase(std::remove_if(buf.begin(),buf.end(),[](const PointT& p) {
    return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z);
  }),buf.end());
  if(buf.empty()) return;
  if (tree_.Root_Node == nullptr) {
    // Build does not downsample. Seed the tree, then apply the same native
    // voxel representative selection used by subsequent insert calls.
    PointVec seed{buf.front()};
    tree_.Build(seed);
    buf.erase(buf.begin());
    if(!buf.empty()) tree_.Add_Points(buf, /*downsample_on=*/true);
  } else {
    tree_.Add_Points(buf, /*downsample_on=*/true);
  }

  last_insert_ns_.store(nowNs() - t0, std::memory_order_relaxed);
}

void IkdTreeMap::insertNoDownsample(const PointVec& world_pts) {
  if (world_pts.empty()) return;
  const uint64_t t0 = nowNs();
  PointVec buf = world_pts;
  buf.erase(std::remove_if(buf.begin(),buf.end(),[](const PointT& p) {
    return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z);
  }),buf.end());
  if(buf.empty()) return;
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
  if (q.k <= 0 || !std::isfinite(q.center.x) || !std::isfinite(q.center.y) ||
      !std::isfinite(q.center.z) || std::isnan(q.max_dist) || q.max_dist < 0 || empty())
    return 0;
  const int k = std::min(q.k, 8);

  PointVec            nearest;
  std::vector<float>  dists;
  nearest.reserve(k);
  dists.reserve(k);

  tree_.Nearest_Search(q.center, k, nearest, dists,
                       q.max_dist == 0 ? std::numeric_limits<float>::epsilon() :
                       std::nextafter(q.max_dist, INFINITY));

  int n = 0;
  const double max2 = static_cast<double>(q.max_dist) * q.max_dist;
  for (std::size_t i = 0; i < nearest.size() && i < dists.size(); ++i) {
    if (!std::isfinite(dists[i]) || dists[i] > max2) continue;
    out[n].p = nearest[i];
    out[n].dist2 = dists[i];
    out[n].voxel_id = 0;
    ++n;
  }
  std::sort(out.begin(), out.begin() + n,
            [](const Neighbor& a, const Neighbor& b) { return a.dist2 < b.dist2; });

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
  if (!std::isfinite(voxel)) throw std::invalid_argument("non-finite snapshot voxel");
  PointVec all;
  tree_.flatten(tree_.Root_Node, all, NOT_RECORD);
  CloudPtr cloud(new CloudT());
  cloud->reserve(all.size());
  for (auto& p : all) cloud->push_back(p);
  if (voxel <= 0 || cloud->empty()) return cloud;
  pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(voxel, voxel, voxel);
  filter.setInputCloud(cloud);
  CloudPtr filtered(new CloudT());
  filter.filter(*filtered);
  return filtered;
}

void IkdTreeMap::flattenInto(PointVec& out) const {
  tree_.flatten(tree_.Root_Node, out, NOT_RECORD);
}

int IkdTreeMap::removeBoxes(const std::vector<BoundingBox>& boxes) {
  for (const auto& b : boxes)
    if (!b.min.allFinite() || !b.max.allFinite() || (b.min.array() > b.max.array()).any())
      throw std::invalid_argument("invalid removal box");
  int count = 0;
  for (const auto& b : boxes) {
    PointVec removed;
    const auto box = toBoxPoint(b);
    tree_.Box_Search(box, removed);
    std::vector<BoxPointType> bp{box};
    count += tree_.Delete_Point_Boxes(bp);
    recordRemoved(removed);
  }
  return count;
}

void IkdTreeMap::cropAround(const Eigen::Vector3d& c, float radius) {
  if (!c.allFinite() || !std::isfinite(radius) || radius <= 0)
    throw std::invalid_argument("invalid crop sphere");
  if (empty()) return;
  const auto range = tree_.tree_range();
  const Eigen::Vector3f lo = (c.array() - radius).matrix().cast<float>();
  const Eigen::Vector3f hi = (c.array() + radius).matrix().cast<float>();
  // Delete the six exterior slabs by tree ranges. Inspect only the shell
  // outside the inscribed cube; never flatten/copy the whole map per frame.
  std::vector<BoundingBox> exterior;
  for (int axis = 0; axis < 3; ++axis) {
    BoundingBox lower, upper;
    for (int i = 0; i < 3; ++i) {
      lower.min[i] = upper.min[i] = range.vertex_min[i];
      lower.max[i] = upper.max[i] = range.vertex_max[i];
    }
    lower.max[axis] = std::nextafter(lo[axis], -INFINITY);
    upper.min[axis] = std::nextafter(hi[axis], INFINITY);
    if ((lower.min.array() <= lower.max.array()).all()) exterior.push_back(lower);
    if ((upper.min.array() <= upper.max.array()).all()) exterior.push_back(upper);
  }
  removeBoxes(exterior);
  const float inner = radius / std::sqrt(3.0f);
  PointVec candidates;
  for (int axis = 0; axis < 3; ++axis)
    for (int sign : {-1, 1}) {
      BoundingBox shell{lo, hi};
      for (int i = 0; i < axis; ++i) {
        shell.min[i] = static_cast<float>(c[i]) - inner;
        shell.max[i] = static_cast<float>(c[i]) + inner;
      }
      if (sign < 0) shell.max[axis] = static_cast<float>(c[axis]) - inner;
      else shell.min[axis] = static_cast<float>(c[axis]) + inner;
      PointVec slab;
      tree_.Box_Search(toBoxPoint(shell), slab);
      for (const auto& p : slab)
        if ((Eigen::Vector3d(p.x, p.y, p.z) - c).squaredNorm() >
            static_cast<double>(radius) * radius) candidates.push_back(p);
    }
  // Boundary overlaps between slabs may contain the same point twice.
  std::sort(candidates.begin(), candidates.end(), [](const PointT& a, const PointT& b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  });
  candidates.erase(std::unique(candidates.begin(), candidates.end(),
      [](const PointT& a, const PointT& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z;
      }), candidates.end());
  // A closed single-coordinate box removes every raw duplicate. Native
  // Delete_Points uses a tolerance and may remove a different nearby point.
  std::vector<BoundingBox> points;
  points.reserve(candidates.size());
  for(const auto& p:candidates) {
    const Eigen::Vector3f xyz(p.x,p.y,p.z);
    points.push_back({xyz,xyz});
  }
  removeBoxes(points);
}

void IkdTreeMap::drainRemoved(PointVec& out) {
  out.insert(out.end(), removed_.begin(), removed_.end());
  removed_.clear();
  PointVec rebuild_removed;
  tree_.acquire_removed_points(rebuild_removed);
}

void IkdTreeMap::recordRemoved(const PointVec& points) {
  for (const auto& p : points) {
    if (removed_.size() >= cfg_.removed_capacity) {
      removed_.pop_front();
      ++removed_dropped_;
    }
    removed_.push_back(p);
  }
}

bool        IkdTreeMap::empty()     const { return tree_.Root_Node == nullptr || tree_.validnum() == 0; }
std::size_t IkdTreeMap::size()      const { return static_cast<std::size_t>(tree_.size()); }
std::size_t IkdTreeMap::validSize() const {
  int count;
  while ((count=tree_.validnum())<0) std::this_thread::yield();
  return static_cast<std::size_t>(count);
}

MapStats IkdTreeMap::stats() const {
  MapStats s;
  s.num_points     = validSize();
  s.removed_dropped = removed_dropped_;
  s.num_voxels     = 0;  // ikd-tree 不按 voxel 计数
  s.bytes          = 0;
  s.last_insert_ms = last_insert_ns_.load(std::memory_order_relaxed) * 1e-6;
  s.last_query_ms  = last_query_ns_.load(std::memory_order_relaxed) * 1e-6;
  return s;
}

std::unique_ptr<IMapManager> makeIkdTreeMap() { return std::make_unique<IkdTreeMap>(); }

}  // namespace slam::map
