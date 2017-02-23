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

#include "squirrel_2d_localizer/laser_model.h"

#include <cmath>
#include <limits>
#include <thread>

namespace squirrel_2d_localizer {

void LaserModel::computeParticlesLikelihood(
    const GridMap& grid_map, const LatentModelLikelihoodField& likelihood_field,
    const std::vector<float>& measurement, std::vector<Particle>* particles) {
  std::unique_lock<std::mutex> lock(mtx_);
  prepareLaserReadings(measurement);
  int e_i, e_j;
  for (size_t i = 0; i < particles->size(); ++i) {
    const Pose2d& pose = particles->at(i).pose;
    double weight      = 1.;
    for (size_t j = 0; j < eff_measurement_.size(); ++j) {
      const EndPoint2d& beam_endpoint = eff_measurement_[j];
      const EndPoint2d& e =
          pose.rotation() * beam_endpoint + pose.translation();
      grid_map.pointToIndices(e, &e_i, &e_j);
      weight *= likelihood_field.likelihood(e_i, e_j);
    }
    particles->at(i).weight = weight;
  }
}

void LaserModel::prepareLaserReadings(const std::vector<float>& measurement) {
  eff_measurement_.clear();
  if (measurement.empty())
    return;
  const double fov = laser_params_.angle_max - laser_params_.angle_min;
  const double angle_increment = fov / (measurement.size() - 1);
  const double sq_min_endpoint_dist =
      std::pow(laser_params_.endpoints_min_distance, 2);
  EndPoint2d last_endpoint({-std::numeric_limits<double>::max(), 0.});
  for (size_t i = 0; i < measurement.size(); ++i) {
    const float ray        = measurement[i];
    const double ray_angle = laser_params_.angle_min + i * angle_increment;
    if (!std::isfinite(ray) || ray < laser_params_.range_min ||
        ray > laser_params_.range_max)
      continue;
    EndPoint2d next_endpoint =
        ray * EndPoint2d(std::cos(ray_angle), std::sin(ray_angle));
    if ((last_endpoint - next_endpoint).squaredNorm() < sq_min_endpoint_dist)
      continue;
    eff_measurement_.emplace_back(
        laser_params_.tf_r2l.rotation() * next_endpoint +
        laser_params_.tf_r2l.translation());
    last_endpoint = next_endpoint;
  }
}

}  // namespace squirrel_2d_localizer
