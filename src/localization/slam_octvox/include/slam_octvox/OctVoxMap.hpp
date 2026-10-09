
// =====================================================================
// This file is vendored from Super-LIO (ros1 branch), commit tree at
// https://github.com/Liansheng-Wang/Super-LIO.git
// Original Copyright (c) Liansheng Wang, licensed under GPLv3.
// See ../../LICENSE for the full GPLv3 text.
//
// Modifications made for integration into fast_lio_super / slam_octvox:
//   - Removed <execution> (unused) and <filesystem> (only saveMap() used it)
//   - Removed saveMap() method (pulls pcl::io + filesystem, not needed here)
//   - include path "HKNN_list60_gem.h" is resolved via the adapter's -I
//     flags (vendored alongside this file in include/super_lio_vendor/).
//   - No algorithmic changes.
// =====================================================================

#ifndef OctVoxMap_HPP_
#define OctVoxMap_HPP_

#include <set>
#include <list>
#include <queue>
#include <vector>
#include <memory>
#include <cstring>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>

#include "tsl/robin_map.h"
#include "HKNN_list60_gem.h"


namespace LI2Sup{

template<int K, typename Point>
class KNNHeap {
public:
  KNNHeap() : count(0), worst_(0), max_dist2_(0.0f) {
    memset(dist2_, 0, sizeof(dist2_));
  }

  void reset() {
    count = 0;
    worst_ = 0;
    max_dist2_ = 0.0f;
    memset(dist2_, 0, sizeof(dist2_));
  }

  uint8_t count;
  uint8_t worst_;
  float max_dist2_;
  float dist2_[K];
  std::array<Point, K> points_;

  inline void try_insert(float dist2, const Point& pt) {
    const bool not_full = (count < K);
    const bool should_insert = not_full || (dist2 < max_dist2_);
    
    if (should_insert) {
      const uint8_t insert_idx = not_full ? count : worst_;
      
      dist2_[insert_idx] = dist2;
      points_[insert_idx] = pt;
      
      if (not_full) {
        count++;
        if (dist2 > max_dist2_) {
          max_dist2_ = dist2;
          worst_ = insert_idx;
        }
      } else {
        update_worst_unrolled();
      }
    }
  }

private:
  inline void update_worst_unrolled() {
    if constexpr (K != 5) {
      worst_ = 0;
      for (uint8_t i = 1; i < count; ++i)
        if (dist2_[i] > dist2_[worst_]) worst_ = i;
      max_dist2_ = dist2_[worst_];
      return;
    }
    float d0 = dist2_[0], d1 = dist2_[1], d2 = dist2_[2], d3 = dist2_[3], d4 = dist2_[4];
    
    uint8_t idx01 = d0 > d1 ? 0 : 1;
    float max01 = d0 > d1 ? d0 : d1;
    
    uint8_t idx23 = d2 > d3 ? 2 : 3;
    float max23 = d2 > d3 ? d2 : d3;
    
    uint8_t idx0123 = max01 > max23 ? idx01 : idx23;
    float max0123 = max01 > max23 ? max01 : max23;
    
    worst_ = max0123 > d4 ? idx0123 : 4;
    max_dist2_ = max0123 > d4 ? max0123 : d4;
  }

public:
  inline float max_dist2() const { return max_dist2_; }
};


template<typename Point>
class OctVox{
public:
  OctVox() { counts_.fill(UNINIT_MASK); }
  OctVox(const Point& pt, uint8_t local_idx)
  {
    counts_.fill(UNINIT_MASK);
    points_[local_idx] = pt;
    counts_[local_idx] = 1;
  }

  ~OctVox() {}

  bool AddPoint(const Point& pt, uint8_t local_idx, bool mean = true,
                int max_representatives = 8) {
    uint8_t& count = counts_[local_idx];
    Point& stored_point = points_[local_idx];
    if(count == UNINIT_MASK) {
      if (std::count_if(counts_.begin(), counts_.end(),
                       [](uint8_t n) { return n != UNINIT_MASK; }) >= max_representatives)
        return false;
      stored_point = pt;
      count = 1;
      return true;
    }

    if(!mean || count >= MAX_POINTS_PER_SUBVOXEL) return false;
    if ((pt - stored_point).squaredNorm() > DISTANCE_THRESHOLD_SQ) return false;

    stored_point = (stored_point * count + pt) / (count + 1);
    ++count;
    return false;
  }

  bool getPoint(const uint8_t local_idx, Point& pt) const {
    if (counts_[local_idx] == UNINIT_MASK) return false;
    pt = points_[local_idx];
    return true;
  }

  static constexpr uint8_t UNINIT_MASK = 0x00;
  static constexpr uint8_t MAX_POINTS_PER_SUBVOXEL = 20;
  static constexpr double DISTANCE_THRESHOLD_SQ = 0.1 * 0.1;

  std::array<uint8_t, 8> counts_;
  std::array<Point, 8> points_;
  std::deque<Point, Eigen::aligned_allocator<Point>> raw_;
};



template<typename Point, typename Scalar>
class OctVoxMap {
public:
  using Ptr = std::shared_ptr<OctVoxMap>;
  using KEY = Eigen::Vector3i;
  using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;
  using KNNHeapType = KNNHeap<8, Point>;
  using OctVoxType = OctVox<Point>;

  struct Options {
    float resolution      = 0.5;   
    std::size_t capacity  = 1000000;
    bool enable_lru = true;
    bool incremental_mean = true;
    int max_representatives = 8;
    std::size_t raw_capacity = 200000;
    std::size_t removed_capacity = 10000;

    Options(float __resolution, std::size_t __capacity) {
      resolution = __resolution;
      capacity = __capacity;
    }
  };

  

  OctVoxMap() {
    SetOptions(options_);
    flat_search_ptrs_.reserve(flat_search_order_offsets.size());
    for(std::size_t i = 0; i < flat_search_order_offsets.size(); i++){
      uint16_t start = flat_search_order_offsets[i];
      flat_search_ptrs_.push_back(const_cast<uint8_t*>(flat_search_order.data() + start));
    }
    group_idx_max_ = flat_search_order_offsets.size() - 1;
  }
  
  ~OctVoxMap() {
    grids_.clear();
    data_.clear();
  }
  
  OctVoxMap(Options options){
    SetOptions(options);
    // std::cout << " ---> OctVoxMap init. Resolution: " << resolution_ 
    //           << " Capacity: " << capacity_ << std::endl;
    flat_search_ptrs_.reserve(flat_search_order_offsets.size());
    for(std::size_t i = 0; i < flat_search_order_offsets.size(); i++){
      uint16_t start = flat_search_order_offsets[i];
      flat_search_ptrs_.push_back(const_cast<uint8_t*>(flat_search_order.data() + start));
    }
    group_idx_max_ = flat_search_order_offsets.size() - 1;
  }

  void SetOptions(const Options& options)
  {
    if (!std::isfinite(options.resolution) || options.resolution <= 0 ||
        options.capacity == 0 || options.raw_capacity == 0 ||
        options.removed_capacity == 0 || options.max_representatives < 1 ||
        options.max_representatives > 8)
      throw std::invalid_argument("invalid OctVox options");
    if (!data_.empty() && options.resolution != resolution_)
      throw std::invalid_argument("cannot change resolution of a populated OctVox map");
    options_ = options;
    resolution_ = options.resolution;
    capacity_ = options.capacity;
    inv_resolution_ = 1.0 / resolution_;
    sub_resolution_ = resolution_ / 2.0;
    sub_inv_resolution_ = 1.0 / sub_resolution_;
    trim();
  }

  void insert(const Points& cloud_world);
  void insertRaw(const Points& cloud_world);
  template<typename Predicate> std::size_t removeIf(Predicate predicate);
  void drainRemoved(Points& out) {
    out.insert(out.end(), removed_.begin(), removed_.end());
    removed_.clear();
  }
  std::size_t point_count() const { return point_count_; }
  std::size_t raw_count() const { return raw_count_; }
  std::size_t removed_dropped() const { return removed_dropped_; }
  // HKNN supplies a distance upper bound; the voxel range completes an exact
  // search. Dense finite-radius queries visit only nearby hash buckets.
  void nearest(const Point& point, int k, float max_dist, KNNHeapType& out) const;
  void printInfo() const;
  void getMap(std::vector<float>&) const;
  // saveMap() removed (was filesystem+pcl::io dependent). Use getMap() +
  // external PCL writer if you need on-disk snapshots.
  void resetMap(const std::vector<float>&);
  void clear();

  void getTopK(const Point& point, KNNHeapType& top_K) const;

  void getTopK_VN(const Point& point, KNNHeapType& top_K) const;

  void reset_max_group(){
    group_idx_max_ = flat_search_order_offsets.size() - 1;
  }

  void decrease_max_group(){
    if(group_idx_max_ > 4) group_idx_max_--;
  }

  // size_t getMemoryUsageBytes() const {
  //   size_t bytes = 0;
  //   bytes += sizeof(*this);
  //   bytes += data_.size() * (sizeof(KEY) + sizeof(OctVoxType)
  //                           + sizeof(void*) * 2); // list node pointers
  //   bytes += grids_.size() * (sizeof(KEY) + sizeof(DATA_ITER)
  //                             + sizeof(size_t)); // hash & pair overhead
  //   bytes += grids_.bucket_count() * sizeof(void*); // bucket array
  //   bytes += flat_search_ptrs_.capacity() * sizeof(uint8_t*);
  //   return bytes;
  // }

  // =========================================================================
  // slam_octvox modification: cheap O(1) getters for the adapter's stats()
  // path. Upstream exposes only printInfo() which goes to stdout; we need
  // non-logging accessors to avoid an O(N) getMap() every time validSize()
  // is called.
  std::size_t voxel_count() const { return data_.size(); }
  std::size_t capacity()    const { return capacity_; }
  float       resolution()  const { return resolution_; }
  // =========================================================================


private:
  Options options_{0.5f, 1000000};
  std::size_t point_count_ = 0, raw_count_ = 0, removed_dropped_ = 0;
  std::deque<Point, Eigen::aligned_allocator<Point>> removed_;
  KEY keyOf(const Point& pt) const {
    return (pt * inv_resolution_).array().floor().template cast<int>();
  }
  bool representable(const Point& pt) const {
    const auto scaled=pt.template cast<double>()*static_cast<double>(sub_inv_resolution_);
    return scaled.allFinite() && (scaled.array().abs()<double(std::numeric_limits<int>::max())-1024).all();
  }
  void recordRemoved(const Point& p) {
    if (removed_.size() == options_.removed_capacity) {
      removed_.pop_front();
      ++removed_dropped_;
    }
    removed_.push_back(p);
  }
  void trim();
  float resolution_ = 0.5;
  float inv_resolution_ = 1.0;
  float sub_resolution_ = 0.25;
  float sub_inv_resolution_ = 4.0;
  std::size_t capacity_ = 1000000;

  bool reset_map_ = false;
  int reset_map_count_ = 0;

  const KEY nearby_grids_[19] = {
    KEY(0, 0, 0),
    KEY(-1, -1, 0), KEY(-1, 0, 0), KEY(-1, 1, 0), 
    KEY(0, -1, 0), KEY(0, 1, 0), 
    KEY(1, -1, 0), KEY(1, 0, 0), KEY(1, 1, 0), 
    KEY(0, 0, -1), KEY(1, 0, -1), KEY(-1, 0, -1), 
    KEY(0, 1, -1), KEY(0, -1, -1), 
    KEY(0, 0, 1), KEY(1, 0, 1), KEY(-1, 0, 1), 
    KEY(0, 1, 1), KEY(0, -1, 1)
  };

  /// HashShiftMix
  struct HASH_VEC {
    std::size_t operator()(const KEY &v) const {
      size_t h = static_cast<size_t>(v[0]);
      h ^= v[1] * 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= v[2] * 0x85ebca6b + (h << 6) + (h >> 2);
      return h;
    }
  };

  using DATA_LIST = std::list<std::pair<KEY, OctVoxType>>;
  using DATA_ITER = typename DATA_LIST::iterator;

  DATA_LIST data_;
  tsl::robin_map<KEY, DATA_ITER, HASH_VEC> grids_;

  std::vector<uint8_t*> flat_search_ptrs_;
  int group_idx_max_;

};


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::insert(const Points& cloud_world){
  if(reset_map_){
    reset_map_count_--;
    if(reset_map_count_ > 0){
      std::cout << "OctVoxMap::insert skip: reset_map_count_ = " << reset_map_count_ << std::endl;
      return;
    } 
    reset_map_ = false;
  }

  for(auto& pt : cloud_world){
    if (!representable(pt)) continue;
    KEY fine_key = (pt * sub_inv_resolution_).array().floor().template cast<int>();
    KEY key;
    key[0] = fine_key[0] >> 1;
    key[1] = fine_key[1] >> 1;
    key[2] = fine_key[2] >> 1;

    uint8_t dx = fine_key[0] & 1;
    uint8_t dy = fine_key[1] & 1;
    uint8_t dz = fine_key[2] & 1;
    uint8_t local_idx = (dz << 2) | (dy << 1) | dx;

    auto iter = grids_.find(key);
    if (iter == grids_.end()) {
      data_.emplace_front(std::piecewise_construct,
        std::forward_as_tuple(key),
        std::forward_as_tuple(pt, local_idx));
      grids_.insert(std::make_pair(key, data_.begin()));
      ++point_count_;
    } else {
      point_count_ += iter->second->second.AddPoint(
          pt, local_idx, options_.incremental_mean, options_.max_representatives);
      data_.splice(data_.begin(), data_, iter->second);
    }
    trim();
  }
}

template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::insertRaw(const Points& points) {
  for (const auto& p : points) {
    if (!representable(p)) continue;
    const KEY key = keyOf(p);
    auto found = grids_.find(key);
    if (found == grids_.end()) {
      data_.emplace_front(key, OctVoxType{});
      grids_.insert(std::make_pair(key, data_.begin()));
      found = grids_.find(key);
    }
    found->second->second.raw_.push_back(p);
    ++raw_count_;
    ++point_count_;
    data_.splice(data_.begin(), data_, found->second);
    trim();
  }
}

template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::trim() {
  while (options_.enable_lru && data_.size() > capacity_) {
    auto last = std::prev(data_.end());
    for (uint8_t i = 0; i < 8; ++i)
      if (last->second.counts_[i]) {
        recordRemoved(last->second.points_[i]);
        --point_count_;
      }
    for (const auto& p : last->second.raw_) recordRemoved(p);
    point_count_ -= last->second.raw_.size();
    raw_count_ -= last->second.raw_.size();
    grids_.erase(last->first);
    data_.erase(last);
  }
  // Raw points have an independent FIFO budget even when voxel LRU is off.
  for (auto it = data_.rbegin(); raw_count_ > options_.raw_capacity;) {
    if (it->second.raw_.empty()) { ++it; continue; }
    recordRemoved(it->second.raw_.front());
    it->second.raw_.pop_front();
    --raw_count_;
    --point_count_;
    if (it->second.raw_.empty() &&
        std::none_of(it->second.counts_.begin(), it->second.counts_.end(),
                     [](uint8_t n) { return n != 0; })) {
      grids_.erase(it->first);
      it = typename DATA_LIST::reverse_iterator(data_.erase(std::prev(it.base())));
    }
  }
  while (removed_.size() > options_.removed_capacity) {
    removed_.pop_front();
    ++removed_dropped_;
  }
}

template<typename Point, typename Scalar>
template<typename Predicate>
std::size_t OctVoxMap<Point, Scalar>::removeIf(Predicate predicate) {
  std::size_t count = 0;
  for (auto it = data_.begin(); it != data_.end();) {
    auto& voxel = it->second;
    for (uint8_t i = 0; i < 8; ++i)
      if (voxel.counts_[i] && predicate(voxel.points_[i])) {
        recordRemoved(voxel.points_[i]);
        voxel.counts_[i] = 0;
        ++count;
      }
    for (auto raw = voxel.raw_.begin(); raw != voxel.raw_.end();) {
      if (predicate(*raw)) {
        recordRemoved(*raw);
        raw = voxel.raw_.erase(raw);
        --raw_count_;
        ++count;
      } else ++raw;
    }
    if (voxel.raw_.empty() &&
        std::none_of(voxel.counts_.begin(), voxel.counts_.end(),
                     [](uint8_t n) { return n != 0; })) {
      grids_.erase(it->first);
      it = data_.erase(it);
    } else ++it;
  }
  point_count_ -= count;
  return count;
}

template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::nearest(const Point& p, int k, float max_dist,
                                     KNNHeapType& out) const {
  if (grids_.empty() || !representable(p) || k<1 || k>8 || std::isnan(max_dist) || max_dist<0) return;
  KNNHeapType seed;
  getTopK(p, seed);
  float bound = max_dist * max_dist;
  if (seed.count >= k) {
    std::array<float, 8> distances{};
    std::copy_n(seed.dist2_, seed.count, distances.begin());
    std::sort(distances.begin(), distances.begin() + seed.count);
    bound = std::min(bound, distances[k - 1]);
  }
  auto visit = [&](const OctVoxType& voxel) {
    for (uint8_t i = 0; i < 8; ++i)
      if (voxel.counts_[i]) {
        const float d = (voxel.points_[i] - p).squaredNorm();
        if (d <= bound) out.try_insert(d, voxel.points_[i]);
      }
    for (const auto& raw : voxel.raw_) {
      const float d = (raw - p).squaredNorm();
      if (d <= bound) out.try_insert(d, raw);
    }
  };
  const double r = std::sqrt(static_cast<double>(bound));
  const double width = 2 * r / resolution_ + 3;
  // Unbounded/sparse diagnostic queries may enumerate occupied buckets. The
  // normal finite-distance residual path uses the HKNN bound and hash lookup.
  if (!std::isfinite(r) || width * width * width > grids_.size() * 8.0 ||
      !representable(p-Point::Constant(static_cast<Scalar>(r))) ||
      !representable(p+Point::Constant(static_cast<Scalar>(r)))) {
    for (const auto& entry : data_) {
      const Point lo = entry.first.template cast<Scalar>() * resolution_;
      const Point delta = (lo - p).cwiseMax(Point::Zero()) +
          (p - lo - Point::Constant(resolution_)).cwiseMax(Point::Zero());
      if (delta.squaredNorm() <= bound) visit(entry.second);
    }
    return;
  }
  const KEY lo = keyOf(p - Point::Constant(static_cast<Scalar>(r)));
  const KEY hi = keyOf(p + Point::Constant(static_cast<Scalar>(r)));
  for (int x = lo.x(); x <= hi.x(); ++x)
    for (int y = lo.y(); y <= hi.y(); ++y)
      for (int z = lo.z(); z <= hi.z(); ++z) {
        auto found = grids_.find(KEY(x, y, z));
        if (found != grids_.end()) visit(found->second->second);
      }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::getTopK(const Point& point, KNNHeapType& top_K) const {
  const KEY fine_key = (point * sub_inv_resolution_).array().floor().template cast<int>();
  KEY key;
  key[0] = fine_key[0] >> 1;
  key[1] = fine_key[1] >> 1;
  key[2] = fine_key[2] >> 1;

  const int dx = fine_key[0] & 1;
  const int dy = fine_key[1] & 1;
  const int dz = fine_key[2] & 1;
  const int local_idx = (dz << 2) | (dy << 1) | dx;
  const KEY mirror_axis = KEY(1 - (dx << 1), 1 - (dy << 1), 1 - (dz << 1));
  
  const int pre_voxel_ptr_size = 8;
  OctVoxType* top_voxels_2_search[pre_voxel_ptr_size];
  std::fill_n(top_voxels_2_search, pre_voxel_ptr_size, nullptr);
  
  for(uint8_t i = 0; i < pre_voxel_ptr_size; ++i)
  {
    KEY delta_key = mirror_axis.cwiseProduct(HKNN_neighbor_voxel[i]);
    KEY n_key = key + delta_key;
    if (auto iter = grids_.find(n_key); iter != grids_.end()) {
      top_voxels_2_search[i] = &iter->second->second;
    }
  }

  Point __sub_point;

  for (int group_idx = 0; group_idx < group_idx_max_; ++group_idx) {
    const uint8_t* group_it = flat_search_ptrs_[group_idx];
    const uint8_t* group_end = flat_search_ptrs_[group_idx + 1];

    while(group_it < group_end){
      const uint8_t neighbor_idx = *group_it++;
      uint8_t data_size = *group_it++;
      
      if(neighbor_idx < pre_voxel_ptr_size)
      {
        OctVoxType* voxel_ptr = top_voxels_2_search[neighbor_idx];
        if (voxel_ptr) {
          while (data_size--) {
            uint8_t _local_idx = (*group_it++)^local_idx;
            if (voxel_ptr->getPoint(_local_idx, __sub_point)) {
              const float dist2 = (__sub_point - point).squaredNorm();
              top_K.try_insert(dist2, __sub_point);
            }
          }
        }
        else group_it+=data_size;
        continue;
      }

      KEY delta_key = mirror_axis.cwiseProduct(HKNN_neighbor_voxel[neighbor_idx]);
      const KEY n_key = key + delta_key;

      if (auto iter = grids_.find(n_key); iter != grids_.end()){
        OctVoxType* voxel_ptr = &iter->second->second;
        while (data_size--){
          const uint8_t _local_idx = (*group_it++)^local_idx;
          if (voxel_ptr->getPoint(_local_idx, __sub_point)) {
            float dist2 = (__sub_point - point).squaredNorm();
            top_K.try_insert(dist2, __sub_point);
          }
        }
      }
      else group_it+=data_size;
    }

    if (top_K.count == 8)
      if (top_K.max_dist2_ < orders_min_dis2[group_idx] * resolution_ * resolution_ * 4.0f){
        break;
      }

  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::getTopK_VN(const Point& point, KNNHeapType& top_K) const{
  KEY key = (point * inv_resolution_).array().floor().template cast<int>();

  std::vector<OctVoxType*> voxels_2_search;
  voxels_2_search.reserve(19);
  for(std::size_t i = 0; i < 19; ++i) {
    KEY n_key = key + nearby_grids_[i];
    if (auto iter = grids_.find(n_key); iter != grids_.end()) {
      voxels_2_search.emplace_back(&iter->second->second);
    }
  }

  Point pt;
  for(auto& voxel : voxels_2_search) {
    for(uint8_t _i = 0; _i < 8; ++_i) {
      if(!voxel->getPoint(_i, pt)) continue;
      float dist2 = (pt - point).squaredNorm();
      top_K.try_insert(dist2, pt);
    }
  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::getMap(std::vector<float>& output) const{
  size_t total_points = point_count_;

  output.clear();
  output.reserve(total_points * 3);

  Point point;
  float pcl_point[3];
  for (const auto& voxel_pair : data_) {
    const OctVoxType& voxel = voxel_pair.second;
    for(uint8_t i = 0; i < 8; ++i) {
      if (!voxel.getPoint(i, point)) continue;
      pcl_point[0] = static_cast<float>(point.x());
      pcl_point[1] = static_cast<float>(point.y());
      pcl_point[2] = static_cast<float>(point.z());
      output.push_back(pcl_point[0]);
      output.push_back(pcl_point[1]);
      output.push_back(pcl_point[2]);
    }
    for (const auto& raw : voxel.raw_) {
      output.push_back(raw.x());
      output.push_back(raw.y());
      output.push_back(raw.z());
    }
  }
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::resetMap(const std::vector<float>& input){
  if (input.empty()) return;
  
  clear();
  size_t num_points = input.size() / 3;

  Points cloud_world;
  cloud_world.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    Point point(input[i * 3], input[i * 3 + 1], input[i * 3 + 2]);
    cloud_world.push_back(point);
  }

  insert(cloud_world);

  reset_map_ = true;
  reset_map_count_ = 10;
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::clear() {
  grids_.clear();
  data_.clear();
  point_count_ = raw_count_ = removed_dropped_ = 0;
  removed_.clear();
}


template<typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::printInfo() const {
    std::cout << " ---> OctVoxMap info. Size: " << data_.size() 
              << " Capacity: " << capacity_ << std::endl;
}

}

#endif
