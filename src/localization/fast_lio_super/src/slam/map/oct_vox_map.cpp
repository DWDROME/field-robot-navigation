// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/map/oct_vox_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <pcl/filters/voxel_grid.h>

namespace slam::map {

namespace {
inline uint64_t nowNs() {
  using namespace std::chrono;
  return static_cast<uint64_t>(
      duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}
// PointXYZINormal -> V3
inline OctVoxMapAdapter::V3 toV3(const PointT& p) {
  return {p.x, p.y, p.z};
}
inline PointT fromV3(const OctVoxMapAdapter::V3& v) {
  PointT p;
  p.x = v.x(); p.y = v.y(); p.z = v.z();
  p.intensity = 0.0f;
  p.normal_x = p.normal_y = p.normal_z = 0.0f;
  p.curvature = 0.0f;
  return p;
}
}  // namespace

OctVoxMapAdapter::OctVoxMapAdapter() {
  backend_ = std::make_unique<BackendT>(typename BackendT::Options{0.5f, 1'000'000});
}

OctVoxMapAdapter::~OctVoxMapAdapter() = default;

void OctVoxMapAdapter::configure(const MapConfig& cfg) {
  typename BackendT::Options opts{
      cfg.voxel_size, cfg.capacity};
  if (!std::isfinite(cfg.local_crop_radius) || cfg.local_crop_radius < 0)
    throw std::invalid_argument("invalid local crop radius");
  opts.enable_lru = cfg.enable_lru;
  opts.incremental_mean = cfg.incremental_mean;
  opts.max_representatives = cfg.max_pts_per_vox;
  opts.raw_capacity = cfg.raw_capacity;
  opts.removed_capacity = cfg.removed_capacity;
  backend_->SetOptions(opts);
  cfg_ = cfg;
}

void OctVoxMapAdapter::insert(const PointVec& world_pts) {
  if (world_pts.empty()) return;
  const uint64_t t0 = nowNs();

  // 投射到 Vector3f。OctVoxMap::insert 自带 sub-voxel 均值过滤，无需我们
  // 再做 voxel 下采样；downsample 语义由 Super-LIO 内部的 8×sub-voxel
  // running-mean 承担，与 ikd-tree 的 downsample_on=true 语义大致等价。
  std::vector<V3, Eigen::aligned_allocator<V3>> v3_buf;
  v3_buf.reserve(world_pts.size());
  for (const auto& p : world_pts) v3_buf.emplace_back(toV3(p));
  backend_->insert(v3_buf);

  last_insert_ns_.store(nowNs() - t0, std::memory_order_relaxed);
}

void OctVoxMapAdapter::insertNoDownsample(const PointVec& world_pts) {
  const uint64_t t0 = nowNs();
  BackendT::Points points;
  points.reserve(world_pts.size());
  for (const auto& p : world_pts) points.push_back(toV3(p));
  backend_->insertRaw(points);
  last_insert_ns_.store(nowNs() - t0, std::memory_order_relaxed);
}

int OctVoxMapAdapter::knn(const nn::Query& q, std::array<Neighbor, 8>& out) const {
  // B-1.c：热路径零锁。OctVoxMap::getTopK const 安全，允许并发读。
  const uint64_t t0 = nowNs();
  if (q.k <= 0 || !toV3(q.center).allFinite() ||
      std::isnan(q.max_dist) || q.max_dist < 0) return 0;
  const int k = std::min(q.k, 8);
  HeapT top_k;
  backend_->nearest(toV3(q.center), k, q.max_dist, top_k);

  const int n = std::min<int>(top_k.count, k);
  // HeapT::points_ 与 dist2_ 顺序不保证递增；按 dist2 排序后写回。
  std::array<std::pair<float, V3>, 8> tmp{};
  for (int i = 0; i < top_k.count; ++i) tmp[i] = {top_k.dist2_[i], top_k.points_[i]};
  std::sort(tmp.begin(), tmp.begin() + top_k.count,
            [](auto& a, auto& b) { return a.first < b.first; });
  for (int i = 0; i < n; ++i) {
    out[i].p        = fromV3(tmp[i].second);
    out[i].dist2    = tmp[i].first;
    out[i].voxel_id = 0;
  }

  last_query_ns_.store(nowNs() - t0, std::memory_order_relaxed);
  return n;
}

bool OctVoxMapAdapter::fitPlane(const nn::Query& q, PlaneCoef& out) const {
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

CloudPtr OctVoxMapAdapter::snapshot(float voxel) const {
  if (!std::isfinite(voxel)) throw std::invalid_argument("non-finite snapshot voxel");
  // snapshot 路径是慢路径（落盘/可视化），允许 O(N) getMap。
  std::vector<float> flat;
  backend_->getMap(flat);
  CloudPtr cloud(new CloudT());
  const std::size_t n = flat.size() / 3;
  cloud->reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    PointT p;
    p.x = flat[i * 3 + 0];
    p.y = flat[i * 3 + 1];
    p.z = flat[i * 3 + 2];
    p.intensity = 0.0f;
    p.normal_x = p.normal_y = p.normal_z = 0.0f;
    p.curvature = 0.0f;
    cloud->push_back(p);
  }
  if (voxel <= 0 || cloud->empty()) return cloud;
  pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(voxel, voxel, voxel);
  filter.setInputCloud(cloud);
  CloudPtr result(new CloudT());
  filter.filter(*result);
  return result;
}

void OctVoxMapAdapter::flattenInto(PointVec& out) const {
  // 同 snapshot，慢路径。
  std::vector<float> flat;
  backend_->getMap(flat);
  const std::size_t n = flat.size() / 3;
  out.reserve(out.size() + n);
  for (std::size_t i = 0; i < n; ++i) {
    PointT p;
    p.x = flat[i * 3 + 0];
    p.y = flat[i * 3 + 1];
    p.z = flat[i * 3 + 2];
    p.intensity = 0.0f;
    p.normal_x = p.normal_y = p.normal_z = 0.0f;
    p.curvature = 0.0f;
    out.push_back(p);
  }
}

int OctVoxMapAdapter::removeBoxes(const std::vector<BoundingBox>& boxes) {
  for (const auto& b : boxes)
    if (!b.min.allFinite() || !b.max.allFinite() || (b.min.array() > b.max.array()).any())
      throw std::invalid_argument("invalid removal box");
  return static_cast<int>(backend_->removeIf([&](const V3& p) {
    return std::any_of(boxes.begin(), boxes.end(), [&](const BoundingBox& b) {
      return (p.array() >= b.min.array()).all() && (p.array() <= b.max.array()).all();
    });
  }));
}

void OctVoxMapAdapter::cropAround(const Eigen::Vector3d& c, float radius) {
  if (!c.allFinite() || !std::isfinite(radius) || radius <= 0)
    throw std::invalid_argument("invalid crop sphere");
  const double radius2 = static_cast<double>(radius) * radius;
  backend_->removeIf([&](const V3& p) {
    return (p.cast<double>() - c).squaredNorm() > radius2;
  });
}

void OctVoxMapAdapter::drainRemoved(PointVec& out) {
  BackendT::Points removed;
  backend_->drainRemoved(removed);
  out.reserve(out.size() + removed.size());
  for (const auto& p : removed) out.push_back(fromV3(p));
}

MapStats OctVoxMapAdapter::stats() const {
  MapStats s;
  s.num_voxels     = backend_->voxel_count();
  s.num_points     = backend_->point_count();
  s.raw_points    = backend_->raw_count();
  s.removed_dropped = backend_->removed_dropped();
  s.bytes          = 0;
  s.last_insert_ms = last_insert_ns_.load(std::memory_order_relaxed) * 1e-6;
  s.last_query_ms  = last_query_ns_.load(std::memory_order_relaxed) * 1e-6;
  return s;
}

std::unique_ptr<IMapManager> makeOctVoxMap() {
  return std::make_unique<OctVoxMapAdapter>();
}

}  // namespace slam::map
