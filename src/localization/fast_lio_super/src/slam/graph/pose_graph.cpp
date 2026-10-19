// SPDX-License-Identifier: GPL-3.0-or-later
#include "slam/graph/backend.hpp"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <stdexcept>
#include <set>

namespace slam::graph {
namespace {
gtsam::Pose3 convert(const Pose& p) {
  if (!validPose(p)) throw std::invalid_argument("invalid pose graph transform");
  return {gtsam::Rot3(p.linear()), gtsam::Point3(p.translation())};
}
gtsam::SharedNoiseModel noise(double rotation, double translation) {
  gtsam::Vector6 sigmas;
  sigmas << rotation, rotation, rotation, translation, translation, translation;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}
}
struct PoseGraph::Impl {
  Config cfg;
  gtsam::ISAM2 isam;
  std::vector<Pose> originals;
  std::set<std::pair<std::size_t,std::size_t>> loops;
  static gtsam::ISAM2Params parameters(const Config& cfg) {
    cfg.validate();
    gtsam::ISAM2Params p;
    p.relinearizeThreshold = cfg.relinearize_threshold;
    p.relinearizeSkip = cfg.relinearize_skip;
    return p;
  }
  explicit Impl(const Config& c) : cfg(c), isam(parameters(c)) {}
};
PoseGraph::PoseGraph(const Config& cfg) : impl_(new Impl(cfg)) {}
PoseGraph::~PoseGraph() = default;
void PoseGraph::addOdometry(std::size_t id, const Pose& original) {
  if (id != impl_->originals.size() || id >= impl_->cfg.max_keyframes)
    throw std::invalid_argument("pose graph key IDs must be consecutive and within budget");
  const auto current = convert(original);
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;
  if (id == 0) {
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(id,current,noise(1e-6,1e-6));
    values.insert(id,current);
  } else {
    const auto delta = convert(impl_->originals.back().inverse()*original);
    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(id-1,id,delta,
      noise(impl_->cfg.odom_rotation_sigma,impl_->cfg.odom_translation_sigma));
    values.insert(id,impl_->isam.calculateEstimate<gtsam::Pose3>(id-1).compose(delta));
  }
  impl_->isam.update(factors,values);
  impl_->isam.update();
  impl_->originals.push_back(original);
}
void PoseGraph::addLoop(std::size_t target, std::size_t source, const Pose& relative) {
  if (target >= impl_->originals.size() || source >= impl_->originals.size() || target == source)
    throw std::invalid_argument("invalid loop key IDs");
  if (impl_->loops.count({target,source})) return;
  gtsam::NonlinearFactorGraph factors;
  auto robust = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(1),
    noise(impl_->cfg.loop_rotation_sigma,impl_->cfg.loop_translation_sigma));
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(target,source,convert(relative),robust);
  impl_->isam.update(factors,gtsam::Values{});
  for (int i = 0; i < 3; ++i) impl_->isam.update();
  impl_->loops.insert({target,source});
}
Pose PoseGraph::estimate(std::size_t id) const {
  Pose pose = Pose::Identity();
  pose.matrix() = impl_->isam.calculateEstimate<gtsam::Pose3>(id).matrix();
  return pose;
}
} // namespace slam::graph
