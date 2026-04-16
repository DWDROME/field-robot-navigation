// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/map/oct_vox_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace slam::map {

namespace {
inline double nowMs() {
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
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
  // 先用默认 options 构造；configure() 会重建以应用 yaml 参数。
  backend_ = std::make_unique<BackendT>(typename BackendT::Options{0.5f, 1'000'000});
}

OctVoxMapAdapter::~OctVoxMapAdapter() = default;

void OctVoxMapAdapter::configure(const MapConfig& cfg) {
  cfg_ = cfg;
  typename BackendT::Options opts{
      (cfg.voxel_size > 0.f ? cfg.voxel_size : 0.5f),
      (cfg.capacity   > 0  ? cfg.capacity   : 1'000'000)};
  backend_->SetOptions(opts);
}

void OctVoxMapAdapter::insert(const PointVec& world_pts) {
  if (world_pts.empty()) return;
  const double t0 = nowMs();

  // 投射到 Vector3f。OctVoxMap::insert 自带 sub-voxel 均值过滤，无需我们
  // 再做 voxel 下采样；downsample 语义由 Super-LIO 内部的 8×sub-voxel
  // running-mean 承担，与 ikd-tree 的 downsample_on=true 语义大致等价。
  std::vector<V3, Eigen::aligned_allocator<V3>> v3_buf;
  v3_buf.reserve(world_pts.size());
  for (const auto& p : world_pts) v3_buf.emplace_back(toV3(p));
  backend_->insert(v3_buf);

  std::lock_guard<std::mutex> lk(stats_mu_);
  stats_.last_insert_ms = nowMs() - t0;
  stats_.num_voxels     = validSize();  // 此处即 data_.size()
}

void OctVoxMapAdapter::insertNoDownsample(const PointVec& world_pts) {
  // Super-LIO 没有"不下采样"通道——其 AddPoint 会拒绝与当前均值距离 >0.1m
  // 的点，这本身就是一种几何稳健性过滤。我们把 NoDownsample 的点一样交给
  // insert：效果上 sub-voxel 均值仍会更新，但不会 bypass 过滤器。文档里
  // 标注这点；若下游需要"无损插入"，需要扩展 OctVox backend。
  insert(world_pts);
}

int OctVoxMapAdapter::knn(const nn::Query& q, std::array<Neighbor, 8>& out) const {
  const double t0 = nowMs();

  HeapT top_k;
  backend_->getTopK(toV3(q.center), top_k);

  const int n = std::min<int>(top_k.count, std::min(q.k, 5));
  // HeapT::points_ 与 dist2_ 顺序不保证递增；我们按 dist2 排序后写回。
  std::array<std::pair<float, V3>, 5> tmp{};
  for (int i = 0; i < top_k.count; ++i) tmp[i] = {top_k.dist2_[i], top_k.points_[i]};
  std::sort(tmp.begin(), tmp.begin() + top_k.count,
            [](auto& a, auto& b) { return a.first < b.first; });
  for (int i = 0; i < n; ++i) {
    out[i].p        = fromV3(tmp[i].second);
    out[i].dist2    = tmp[i].first;
    out[i].voxel_id = 0;
  }

  {
    std::lock_guard<std::mutex> lk(stats_mu_);
    stats_.last_query_ms = nowMs() - t0;
  }
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

CloudPtr OctVoxMapAdapter::snapshot(float /*voxel*/) const {
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
  return cloud;
}

void OctVoxMapAdapter::flattenInto(PointVec& out) const {
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

int OctVoxMapAdapter::removeBoxes(const std::vector<BoundingBox>& /*boxes*/) {
  // Super-LIO 不支持 box-delete：地图体积控制靠 LRU。保持为 no-op 是有意
  // 选择；若调用方确实需要 FoV 立方体删除，应切回 ikdtree 后端。
  return 0;
}

void OctVoxMapAdapter::cropAround(const Eigen::Vector3d& /*c*/, float /*radius*/) {
  // Super-LIO 无显式 radius crop，同 removeBoxes：LRU 承担。
}

void OctVoxMapAdapter::drainRemoved(PointVec& /*out*/) {
  // Super-LIO 不追踪被 LRU 淘汰的点。返回空集合。
}

bool OctVoxMapAdapter::empty() const {
  // 借 validSize 判断：Super-LIO 内部 data_ 为空时没有 API 直接暴露，
  // 用 validSize==0 间接判断。
  return validSize() == 0;
}

std::size_t OctVoxMapAdapter::size() const {
  return validSize();  // voxel 级计数，与 validSize 相同
}

std::size_t OctVoxMapAdapter::validSize() const {
  std::vector<float> flat;
  backend_->getMap(flat);
  return flat.size() / 3;  // 代表点总数
}

MapStats OctVoxMapAdapter::stats() const {
  std::lock_guard<std::mutex> lk(stats_mu_);
  MapStats s = stats_;
  s.num_points = validSize();
  return s;
}

std::unique_ptr<IMapManager> makeOctVoxMap() {
  return std::make_unique<OctVoxMapAdapter>();
}

}  // namespace slam::map
