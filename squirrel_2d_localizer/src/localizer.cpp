// The MIT License (MIT)
//
// Copyright (c) 2017 Federico Boniardi and Wolfram Burgard
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
    std::unique_ptr<GridMap>& map,
    std::unique_ptr<LatentModelLikelihoodField>& likelihood_field,
    std::unique_ptr<LaserModel>& laser_model,
    std::unique_ptr<MotionModel>& motion_model) {
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
  particles_.reserve(params_.num_particles);
  particles_.emplace_back(init_pose, 1.);
  for (size_t i = 1; i < params_.num_particles; ++i) {
    const double x = init_pose[0] + params_.init_stddev_x * randn(rnd_eng);
    const double y = init_pose[1] + params_.init_stddev_y * randn(rnd_eng);
    const double a = init_pose[2] + params_.init_stddev_a * randn(rnd_eng);
    particles_.emplace_back(Pose2d(x, y, a), 0.);
  }
  pose_       = init_pose;
  covariance_ = Eigen::Matrix3d();
  covariance_(0, 0) = params_.init_stddev_x;
  covariance_(1, 1) = params_.init_stddev_y;
  covariance_(2, 2) = params_.init_stddev_a;
  // reset cummulative motion
  cum_lin_motion_ = 0.;
  cum_ang_motion_ = 0.;
}

void Localizer::resetParticles(const std::vector<Particle>& particles) {
  std::unique_lock<std::mutex> lock(init_mtx_);
  // Reset particle set.
  if (!particles_.empty())
    particles_.clear();
  particles_ = particles;
  // Update pose and covariance.
  particles::computeMeanAndCovariance(particles_, &pose_, &covariance_);
  // reset cummulative motion.
  cum_lin_motion_ = 0.;
  cum_ang_motion_ = 0.;
}

bool Localizer::updateFilter(
    const Transform2d& motion, const std::vector<float>& scan,
    const Transform2d& extra_correction) {
  std::unique_lock<std::mutex> lock(init_mtx_);
  cum_lin_motion_ += motion.translation().norm();
  cum_ang_motion_ += std::abs(angles::normalize_angle(motion[2]));
  if (cum_lin_motion_ < params_.min_lin_update &&
      cum_ang_motion_ < params_.min_ang_update)
    return false;
  motion_model_->sampleProposal(motion, &particles_);
  laser_model_->computeParticlesLikelihood(
      *map_, *likelihood_field_, scan, &particles_);
  resampling::importanceSampling(&particles_);
  particles::computeMeanAndCovariance(particles_, &pose_, &covariance_);
  pose_ *= extra_correction;
  cum_lin_motion_ = 0.;
  cum_ang_motion_ = 0.;
  return true;
}

Localizer::Params Localizer::Params::defaultParams() {
  Params params;
  params.num_particles  = 250;
  params.min_lin_update = 1.0;
  params.min_ang_update = 1.0;
  return params;
}

}  // namespace squirrel_2d_localizer
