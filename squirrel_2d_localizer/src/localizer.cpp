// The MIT License (MIT)
//
// Copyright (c) 2016 Federico Boniardi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "squirrel_2d_localizer/localizer.h"
#include "squirrel_2d_localizer/resampling.h"

#include <random>
#include <thread>
#include <utility>

namespace squirrel_2d_localizer {

void Localizer::initialize(
    GridMap::Ptr& map, LikelihoodField::Ptr& likelihood_field,
    LaserModel::Ptr& laser_model, MotionModel::Ptr& motion_model) {
  map_              = std::move(map);
  likelihood_field_ = std::move(likelihood_field);
  laser_model_      = std::move(laser_model);
  motion_model_     = std::move(motion_model);
}

void Localizer::resetPose(const Pose2d& init_pose) {
  std::unique_lock<std::mutex> lock(init_mtx_);
  std::mt19937 rnd_eng(std::rand());
  std::normal_distribution<double> randn(0., 1.);
  if (!particles_.empty())
    particles_.clear();
  particles_.reserve(loc_params_.num_particles);
  particles_.emplace_back(init_pose, 1.);
#pragma omp parallel for default(shared)
  for (size_t i = 1; i < loc_params_.num_particles; ++i) {
    const double x = init_pose[0] + loc_params_.init_stddev_x * randn(rnd_eng);
    const double y = init_pose[1] + loc_params_.init_stddev_y * randn(rnd_eng);
    const double a = init_pose[2] + loc_params_.init_stddev_a * randn(rnd_eng);
    particles_.emplace_back(Pose2d(x, y, a), 0.);
  }
  pose_       = init_pose;
  covariance_ = Matrix<3, 3>();
  covariance_(0, 0) = loc_params_.init_stddev_x;
  covariance_(1, 1) = loc_params_.init_stddev_y;
  covariance_(2, 2) = loc_params_.init_stddev_a;
  // reset cummulative motion
  cum_lin_motion_ = 0.;
  cum_ang_motion_ = 0.;
}

bool Localizer::updateFilter(
    const Transform2d& motion, const std::vector<float>& scan,
    const Transform2d& extra_correction) {
  cum_lin_motion_ += motion.translation().norm();
  cum_ang_motion_ += std::abs(angles::normalize_angle(motion[2]));
  if (cum_lin_motion_ < loc_params_.min_lin_update &&
      cum_ang_motion_ < loc_params_.min_ang_update)
    return false;
  motion_model_->propagateParticles(motion, &particles_);
  laser_model_->computeParticlesLikelihood(
      *map_, *likelihood_field_, scan, &particles_);
  resampling::importanceSampling(&particles_);
  particles::computeMeanAndCovariance(particles_, &pose_, &covariance_);
  pose_ *= extra_correction;
  cum_lin_motion_ = 0.;
  cum_ang_motion_ = 0.;
  return true;
}

}  // namespace squirrel_2d_localizer
