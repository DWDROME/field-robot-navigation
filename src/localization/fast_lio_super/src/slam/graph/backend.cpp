// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/graph/backend.hpp"
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <cmath>

namespace slam::graph {
struct Backend::Impl {
  struct Frame { double stamp; Pose pose; Cloud::ConstPtr cloud; };
  Config cfg;
  ScanContext descriptors;
  PoseGraph graph;
  std::vector<Keyframe> frames;
  std::vector<Descriptor> contexts;
  std::size_t cloud_points = 0, accepted = 0, rejected = 0;
  std::atomic<std::size_t> dropped{0};
  double last_stamp = -1, last_candidate = -1, translation = 0, rotation = 0;
  Pose previous = Pose::Identity();
  std::mutex input_mutex;
  mutable std::mutex output_mutex;
  std::condition_variable ready;
  std::deque<Frame> input;
  bool stopping = false;
  std::thread worker;
  std::shared_ptr<const Snapshot> output = std::make_shared<Snapshot>();
  explicit Impl(const Config& c) : cfg(c), descriptors(c), graph(c) {
    worker = std::thread([this] { run(); });
  }
  void publish(const std::string& status) {
    auto state = std::make_shared<Snapshot>();
    state->keyframes = frames;
    if (!frames.empty()) state->map_from_odom = frames.back().optimized*frames.back().original.inverse();
    state->accepted_loops = accepted; state->rejected_loops = rejected;
    state->dropped_frames = dropped.load(); state->status = status;
    std::lock_guard<std::mutex> lock(output_mutex);
    output = std::move(state);
  }
  void process(const Frame& f) {
    if (f.stamp <= last_stamp) { ++dropped; publish("non-monotonic-frame"); return; }
    if (last_stamp >= 0) {
      const Pose delta = previous.inverse()*f.pose;
      translation += delta.translation().norm();
      rotation += Eigen::AngleAxisd(delta.linear()).angle();
    }
    previous = f.pose; last_stamp = f.stamp;
    if (!frames.empty() && translation < cfg.keyframe_translation && rotation < cfg.keyframe_rotation)
      return;
    if (frames.size() >= cfg.max_keyframes) { publish("keyframe-budget-exhausted"); return; }
    auto cloud = downsample(f.cloud,cfg.voxel);
    if (cloud->size() > cfg.max_frame_points) {
      Cloud::Ptr bounded(new Cloud);
      bounded->reserve(cfg.max_frame_points);
      for (std::size_t i = 0; i < cfg.max_frame_points; ++i)
        bounded->push_back((*cloud)[i*cloud->size()/cfg.max_frame_points]);
      cloud = bounded;
    }
    if (cloud->size() < static_cast<std::size_t>(cfg.min_inliers)) {
      ++dropped; publish("insufficient-keyframe-points"); return;
    }
    if (cloud_points + cloud->size() > cfg.max_cloud_points) {
      publish("keyframe-cloud-budget-exhausted"); return;
    }
    const std::size_t id = frames.size();
    graph.addOdometry(id,f.pose);
    frames.push_back({id,f.stamp,f.pose,graph.estimate(id),cloud});
    contexts.push_back(descriptors.describe(*cloud));
    cloud_points += cloud->size();
    translation = rotation = 0;
    std::string status = "odometry-updated";
    if (f.stamp-last_candidate >= cfg.candidate_period) {
      last_candidate = f.stamp;
      std::vector<std::pair<double,std::size_t>> keys;
      for (std::size_t i = 0; i < id; ++i)
        if (id-i >= static_cast<std::size_t>(cfg.min_loop_keyframes) &&
            f.stamp-frames[i].stamp >= cfg.min_loop_seconds)
          keys.emplace_back((contexts.back().ring_key-contexts[i].ring_key).squaredNorm(),i);
      std::sort(keys.begin(),keys.end());
      Candidate best;
      const std::size_t count = std::min<std::size_t>(keys.size(),cfg.candidate_count);
      for (std::size_t i = 0; i < count; ++i) {
        auto candidate = descriptors.compare(contexts.back(),contexts[keys[i].second]);
        if (candidate.distance < best.distance) { best = candidate; best.id = keys[i].second; }
      }
      if (best.distance < cfg.descriptor_threshold) {
        const std::size_t target_id = best.id;
        const Pose target_pose = graph.estimate(target_id);
        Cloud::Ptr submap(new Cloud);
        const std::size_t start = target_id > static_cast<std::size_t>(cfg.submap_frames) ?
          target_id-cfg.submap_frames : 0;
        const std::size_t end = std::min<std::size_t>(id-cfg.min_loop_keyframes,target_id+cfg.submap_frames);
        for (std::size_t i = start; i <= end; ++i)
          if (f.stamp-frames[i].stamp >= cfg.min_loop_seconds)
            *submap += *transformCloud(*frames[i].cloud,target_pose.inverse()*graph.estimate(i));
        auto target = downsample(submap,cfg.voxel);
        Pose initial = target_pose.inverse()*graph.estimate(id);
        const double yaw = std::atan2(initial.linear()(1,0),initial.linear()(0,0));
        initial.linear() = Eigen::AngleAxisd(best.yaw-yaw,Eigen::Vector3d::UnitZ())*initial.linear();
        const auto alignment = verifyGicp(cloud,target,initial,cfg);
        status = alignment.reason;
        if (alignment.accepted) {
          graph.addLoop(target_id,id,alignment.target_from_source);
          ++accepted;
        } else ++rejected;
      }
    }
    for (auto& frame : frames) frame.optimized = graph.estimate(frame.id);
    publish(status);
  }
  void run() {
    while (true) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(input_mutex);
        ready.wait(lock,[&] { return stopping || !input.empty(); });
        if (stopping) return;
        frame = std::move(input.front()); input.pop_front();
      }
      try { process(frame); }
      catch (const std::exception& e) {
        publish(std::string("backend-error: ")+e.what());
        std::lock_guard<std::mutex> lock(input_mutex);
        stopping = true; input.clear(); return;
      }
    }
  }
};
Backend::Backend(const Config& config) : impl_(new Impl(config)) {}
Backend::~Backend() { stop(); }
bool Backend::submit(double stamp, const Pose& pose, Cloud::ConstPtr cloud) {
  if (!std::isfinite(stamp) || stamp <= 0 || !validPose(pose) || !cloud || cloud->empty()) return false;
  for (const auto& p : *cloud)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
  if (cloud->size() > impl_->cfg.max_frame_points) {
    Cloud::Ptr bounded(new Cloud);
    bounded->reserve(impl_->cfg.max_frame_points);
    for (std::size_t i = 0; i < impl_->cfg.max_frame_points; ++i)
      bounded->push_back((*cloud)[i*cloud->size()/impl_->cfg.max_frame_points]);
    cloud = bounded;
  }
  std::lock_guard<std::mutex> lock(impl_->input_mutex);
  if (impl_->stopping) return false;
  if (impl_->input.size() == impl_->cfg.queue_capacity) {
    impl_->input.pop_front(); ++impl_->dropped;
  }
  impl_->input.push_back({stamp,pose,std::move(cloud)});
  impl_->ready.notify_one();
  return true;
}
std::shared_ptr<const Snapshot> Backend::snapshot() const {
  std::lock_guard<std::mutex> lock(impl_->output_mutex);
  return impl_->output;
}
void Backend::stop() {
  {
    std::lock_guard<std::mutex> lock(impl_->input_mutex);
    impl_->stopping = true; impl_->input.clear();
  }
  impl_->ready.notify_all();
  if (impl_->worker.joinable()) impl_->worker.join();
}
} // namespace slam::graph
