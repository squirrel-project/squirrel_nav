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
#pragma omp parallel for default(shared)
  for (size_t i = 0; i < particles->size(); ++i) {
    const Pose2d& pose = particles->at(i).pose;
    double weight      = 1.;
#pragma omp parallel for reduction(* : weight)
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
