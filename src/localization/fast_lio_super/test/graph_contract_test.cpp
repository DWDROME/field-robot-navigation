// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/graph/backend.hpp"
#include <cassert>
#include <chrono>
#include <thread>
#include <iostream>

using namespace slam::graph;
Cloud::Ptr fixture() {
  Cloud::Ptr cloud(new Cloud);
  for (int i = 0; i < 900; ++i) {
    Point p;
    const double a = i*2.399963229728653;
    const double r = 2 + (i%41)*0.19;
    p.x = r*std::cos(a); p.y = r*std::sin(a);
    p.z = std::sin(i*0.73)*1.3 + (i%17)*0.12; p.intensity = i%255;
    cloud->push_back(p);
  }
  return cloud;
}
int main() {
  Config cfg;
  cfg.voxel = 0.05; cfg.min_loop_keyframes = 2; cfg.min_loop_seconds = 2;
  cfg.submap_frames = 0; cfg.max_mean_square_error = 0.05;
  cfg.max_keyframes = 8; cfg.max_frame_points = 2000; cfg.max_cloud_points = 16000;
  auto cloud = fixture();
  ScanContext sc(cfg);
  auto d = sc.describe(*cloud);
  assert(sc.compare(d,d).distance < 1e-6);
  Pose yaw = Pose::Identity();
  yaw.linear() = Eigen::AngleAxisd(6*6.283185307179586/60,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto match = sc.compare(sc.describe(*transformCloud(*cloud,yaw)),d);
  assert(match.distance < 0.03);
  assert(std::fabs(match.yaw+6*6.283185307179586/60) < 0.11);
  Pose shift = Pose::Identity(); shift.translation() = Eigen::Vector3d(0.2,-0.15,0.1);
  auto source = transformCloud(*cloud,shift.inverse());
  const auto good = verifyGicp(source,cloud,shift,cfg);
  assert(good.accepted);
  assert((good.target_from_source.translation()-shift.translation()).norm() < 0.03);
  auto limited=cfg; limited.gicp_iterations=1;
  const auto unfinished=verifyGicp(source,cloud,Pose::Identity(),limited);
  assert(!unfinished.accepted && unfinished.reason=="iteration-limit");
  Pose far = Pose::Identity(); far.translation().x() = 100;
  assert(!verifyGicp(transformCloud(*cloud,far),cloud,Pose::Identity(),cfg).accepted);
  Cloud::Ptr small(new Cloud); small->push_back((*cloud)[0]);
  assert(!verifyGicp(small,cloud,Pose::Identity(),cfg).accepted);
  auto invalid = Cloud::Ptr(new Cloud(*cloud)); invalid->points[0].x = NAN;
  assert(!verifyGicp(invalid,cloud,Pose::Identity(),cfg).accepted);
  PoseGraph graph(cfg);
  Pose p = Pose::Identity();
  graph.addOdometry(0,p);
  p.translation().x() = 1; graph.addOdometry(1,p);
  p.translation().x() = 0.4; graph.addOdometry(2,p);
  const double before = graph.estimate(2).translation().norm();
  graph.addLoop(0,2,Pose::Identity());
  assert(graph.estimate(2).translation().norm() < before);
  assert(graph.estimate(0).translation().norm() < 1e-4);
  Backend backend(cfg);
  assert(backend.submit(1,Pose::Identity(),cloud));
  auto waitFor = [&](std::size_t count) {
    for (int i = 0; i < 500 && backend.snapshot()->keyframes.size() < count; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    assert(backend.snapshot()->keyframes.size() == count);
  };
  waitFor(1);
  p = Pose::Identity(); p.translation().x() = 1.1;
  assert(backend.submit(2,p,source)); waitFor(2);
  p.translation().x() = 0.1;
  assert(backend.submit(3,p,cloud)); waitFor(3);
  const auto state = backend.snapshot();
  assert(state->accepted_loops == 1);
  assert(validPose(state->map_from_odom));
  assert((state->keyframes.back().original.translation()-p.translation()).norm() < 1e-9);
  backend.stop();
  assert(!backend.submit(4,p,cloud));
  Config bounded=cfg; bounded.max_keyframes=3; bounded.min_loop_keyframes=99;
  Backend lifecycle(bounded);
  auto waitStatus=[&](const std::string& status) {
    for(int i=0;i<500 && lifecycle.snapshot()->status!=status;++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    assert(lifecycle.snapshot()->status==status);
  };
  p=Pose::Identity(); assert(lifecycle.submit(1,p,cloud)); waitStatus("odometry-updated");
  for(int i=1;i<=3;++i) {p.translation().x()=i*.4; assert(lifecycle.submit(i+1,p,cloud));}
  for(int i=0;i<500 && lifecycle.snapshot()->keyframes.size()!=2;++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  assert(lifecycle.snapshot()->keyframes.size()==2); // accumulated displacement
  p.linear()=Eigen::AngleAxisd(.21,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  assert(lifecycle.submit(5,p,cloud));
  for(int i=0;i<500 && lifecycle.snapshot()->keyframes.size()!=3;++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  assert(lifecycle.snapshot()->keyframes.size()==3); // actual rotation angle
  p.translation().x()+=1.1; assert(lifecycle.submit(6,p,cloud)); waitStatus("keyframe-budget-exhausted");
  assert(lifecycle.snapshot()->keyframes.size()==3);
  assert(lifecycle.submit(5,p,cloud)); waitStatus("non-monotonic-frame");
  lifecycle.stop(); assert(!lifecycle.submit(7,p,cloud));
  std::cout << "Scan Context, GICP, iSAM2, correction and worker lifecycle passed\n";
}
