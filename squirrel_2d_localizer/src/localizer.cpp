// Copyright (c) 2016-2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "squirrel_2d_localizer/localizer.h"
#include "squirrel_2d_localizer/resampling.h"

#include <random>
#include <thread>
#include <utility>

namespace squirrel_2d_localizer {

void Localizer::initialize(
    GridMap::Ptr& map,
    LatentModelLikelihoodField::Ptr& likelihood_field,
    LaserModel::Ptr& laser_model, MotionModel::Ptr& motion_model) {
  map_              = std::move(map);
  likelihood_field_ = std::move(likelihood_field);
  laser_model_      = std::move(laser_model);
  motion_model_     = std::move(motion_model);
}

void Localizer::resetPose(const Pose2d& init_pose) {
  std::unique_lock<std::mutex> lock(init_mtx);
  std::mt19937 rnd_eng(std::rand());
  std::normal_distribution<double> randn(0., 1.);
  if (!particles_.empty())
    particles_.clear();
  particles_.reserve(loc_params_.num_particles);
  particles_.emplace_back(init_pose, 1.);
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
  std::unique_lock<std::mutex> lock(init_mtx);
  cum_lin_motion_ += motion.translation().norm();
  cum_ang_motion_ += std::abs(angles::normalize_angle(motion[2]));
  if (cum_lin_motion_ < loc_params_.min_lin_update &&
      cum_ang_motion_ < loc_params_.min_ang_update)
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

}  // namespace squirrel_2d_localizer
