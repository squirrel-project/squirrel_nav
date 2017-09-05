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

#include "squirrel_2d_localizer/resampling.h"
#include "squirrel_2d_localizer/particle_types.h"

#include <algorithm>
#include <random>
#include <thread>
#include <vector>

namespace squirrel_2d_localizer {
namespace resampling {

void importanceSampling(std::vector<Particle>* particles) {
  std::unique_lock<std::mutex> lock(__internal::resampling_mtx_);
  std::mt19937 eng(std::rand());
  std::uniform_real_distribution<double> rand(0., 1.);
  const double tot_weights = particles::computeTotalWeight(*particles);
  const size_t nparticles  = particles->size();
  const double interval    = tot_weights / nparticles;
  std::vector<size_t> indexes(nparticles);
  double cum_weight = 0.;
  double target     = interval * rand(eng);
  for (size_t i = 0, j = 0; i < nparticles; ++i) {
    cum_weight += particles->at(i).weight;
    while (cum_weight > target && j < nparticles) {
      indexes[j++] = i;
      target += interval;
    }
  }
  std::vector<Pose2d> new_poses;
  new_poses.reserve(nparticles);
  for (size_t i = 0, idx; i < nparticles; ++i) {
    idx = indexes[i];
    new_poses.emplace_back(particles->at(idx).pose);
  }
  particles::setPoses(new_poses, particles);
  particles::setWeight(1. / nparticles, particles);
}

void uniformUpsample(int nparticles_add, std::vector<Particle>* particles) {
  std::unique_lock<std::mutex> lock(__internal::resampling_mtx_);
  std::mt19937 rg(std::rand());
  std::normal_distribution<double> randn(0, 1);
  // Compute mean and covariance.
  Pose2d mean;
  Eigen::Matrix3d cov;
  particles::computeMeanAndCovariance(*particles, &mean, &cov);
  const Eigen::Vector3d& mean_vec = mean.toVector();
  // Create new particles.
  for (int i = 0; i < nparticles_add; ++i) {
    const double x = randn(rg), y = randn(rg), a = randn(rg);
    particles->emplace_back(
        Pose2d(cov * Eigen::Vector3d(x, y, a) + mean_vec), 0.);
  }
}

void uniformDownsample(
    int nparticles_remove, std::vector<Particle>* particles) {
  std::unique_lock<std::mutex> lock(__internal::resampling_mtx_);
  // Extract uniformly which particles to keep.
  const int new_particles_num = particles->size() - nparticles_remove;
  std::shuffle(particles->begin(), particles->end(), std::mt19937(std::rand()));
  particles->erase(particles->begin() + new_particles_num, particles->end());
}

}  // namespace resampling
}  // namespace squirrel_2d_localizer
