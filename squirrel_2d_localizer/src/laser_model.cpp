// Copyright (c) 2016-2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the University of Freiburg nor the names of
//   its contributors may be used to endorse or promote products
//   derived from this software without specific prior written
//   permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

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
      grid_map.pointToIndices(pose * eff_measurement_[j], &e_i, &e_j);
      weight *= likelihood_field.likelihood(e_i, e_j);
    }
    particles->at(i).weight = weight;
  }
}

void LaserModel::toCartesian(
    const std::vector<float>& ranges,
    std::vector<EndPoint2d, Eigen::aligned_allocator<EndPoint2d>>* endpoints)
    const {
  const int nranges = ranges.size();
  // Get the angles.
  const double amin = params_.angle_min;
  const double ainc = (params_.angle_max - params_.angle_min) / (nranges - 1);
  // Fill the endpoints.
  endpoints->reserve(ranges.size());
  for (int i = 0; i < nranges; ++i) {
    const double a = amin + i * ainc;
    const double r = ranges[i];
    if (r >= params_.range_min && r <= params_.range_max)
      endpoints->emplace_back(
          params_.tf_r2l * EndPoint2d(r * std::cos(a), r * std::sin(a)));
  }
}

void LaserModel::prepareLaserReadings(const std::vector<float>& measurement) {
  std::vector<EndPoint2d, Eigen::aligned_allocator<EndPoint2d>> endpoints;
  // Convert to endpoints.
  eff_measurement_.clear();
  toCartesian(measurement, &endpoints);
  if (endpoints.empty())
    return;
  eff_measurement_.reserve(endpoints.size());
  eff_measurement_.emplace_back(endpoints.front());
  for (const auto& endpoint : endpoints) {
    const auto& eff_endpoint = eff_measurement_.back();
    if ((endpoint - eff_endpoint).norm() >= params_.endpoints_min_distance)
      eff_measurement_.emplace_back(endpoint);
  }
  eff_measurement_.shrink_to_fit();
}

LaserModel::Params LaserModel::Params::defaultParams() {
  Params params;
  params.endpoints_min_distance = 0.5;
  params.range_min              = 0.;
  params.range_max              = 6.;
  params.angle_min              = -0.5 * M_PI;
  params.angle_max              = 0.5 * M_PI;
  return params;
}

}  // namespace squirrel_2d_localizer
