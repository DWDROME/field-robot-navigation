// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/map/map_manager.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

slam::PointT point(float x, float y = 0, float z = 0) {
  slam::PointT p{};
  p.x = x; p.y = y; p.z = z;
  return p;
}

void contract(std::unique_ptr<slam::map::IMapManager> map) {
  slam::map::MapConfig cfg;
  cfg.voxel_size = 0.5f;
  map->configure(cfg);
  std::array<slam::Neighbor, 8> neighbours{};
  slam::nn::Query q;
  q.center = point(0); q.k = 8; q.max_dist = 2;
  assert(map->search().knn(q, neighbours) == 0);
  slam::PointVec points;
  for (int i = 0; i < 8; ++i) points.push_back(point(i * 0.02f));
  points.push_back(point(1));
  points.push_back(point(2));
  points.push_back(point(2, 2));
  map->insertNoDownsample(points);
  assert(map->validSize() == 11);
  assert(map->snapshot()->size() == 11);
  assert(map->snapshot(0.5)->size() < 11);
  assert(map->validSize() == 11);
  assert(map->search().knn(q, neighbours) == 8);
  for (int i = 0; i < 8; ++i) {
    assert(std::fabs(neighbours[i].p.x - i * 0.02f) < 1e-6);
    if (i) assert(neighbours[i-1].dist2 <= neighbours[i].dist2);
  }
  q.k = 99;
  assert(map->search().knn(q, neighbours) == 8);
  for(int k:{1,5}) {q.k=k; assert(map->search().knn(q,neighbours)==k);}
  q.k=8; q.center=point(2); q.max_dist=1;
  assert(map->search().knn(q,neighbours)==2 && neighbours[1].dist2==1); // inclusive boundary
  q.center=point(0);
  q.k = 0; assert(map->search().knn(q, neighbours) == 0);
  q.k = 8; q.max_dist = 0;
  assert(map->search().knn(q, neighbours) == 1);
  q.max_dist = -1; assert(map->search().knn(q, neighbours) == 0);
  q.max_dist = std::numeric_limits<float>::quiet_NaN();
  assert(map->search().knn(q, neighbours) == 0);
  q.max_dist = INFINITY; q.center.x = NAN;
  assert(map->search().knn(q, neighbours) == 0);
  map->cropAround(Eigen::Vector3d::Zero(), 2);
  assert(map->validSize() == 10); // sphere boundary remains; (2,2) is removed
  slam::PointVec removed;
  map->drainRemoved(removed); assert(removed.size() == 1);
  removed.clear(); map->drainRemoved(removed); assert(removed.empty());
  slam::BoundingBox box;
  box.min = Eigen::Vector3f(1, 0, 0); box.max = Eigen::Vector3f(2, 0, 0);
  assert(map->removeBoxes({box, box}) == 2);
  assert(map->validSize() == 8);
  map->drainRemoved(removed); assert(removed.size() == 2);
  assert(map->stats().num_points == 8);
  std::cout << map->name() << " contract passed\n";
  map->insertNoDownsample({point(2,2),point(2,2),point(2,2)});
  map->cropAround(Eigen::Vector3d::Zero(),2);
  assert(map->validSize()==8);
  removed.clear(); map->drainRemoved(removed); assert(removed.size()==3);
}

int main() {
  for(auto factory:{slam::map::makeIkdTreeMap,slam::map::makeOctVoxMap}) {
    auto representative=factory(); slam::map::MapConfig c; representative->configure(c);
    representative->insert({point(.01),point(.02)});
    assert(representative->validSize()==1);
    representative->insert({point(.03)}); assert(representative->validSize()==1);
  }
  contract(slam::map::makeIkdTreeMap());
  contract(slam::map::makeOctVoxMap());
  auto map = slam::map::makeOctVoxMap();
  slam::map::MapConfig cfg;
  cfg.capacity = 1; cfg.raw_capacity = 2; cfg.removed_capacity = 1;
  map->configure(cfg);
  map->insert({point(0)}); assert(map->validSize() == 1);
  map->insert({point(2)}); assert(map->validSize() == 1);
  cfg.enable_lru = false;
  map->configure(cfg);
  map->insert({point(4)}); assert(map->stats().num_voxels == 2);
  map->insertNoDownsample({point(4.01), point(4.02), point(4.03)});
  assert(map->stats().raw_points == 2);
  assert(map->stats().removed_dropped >= 1);
  std::array<slam::Neighbor,8> n{};
  slam::nn::Query q; q.center = point(100); q.k = 8; q.max_dist = INFINITY;
  assert(map->search().knn(q,n) == 4); // sparse unbounded query is complete
  assert(map->validSize() == 4);
  std::cout << "OctVox budgets passed\n";
}
