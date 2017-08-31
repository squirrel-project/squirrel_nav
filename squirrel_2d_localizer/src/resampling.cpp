// The MIT License (MIT)
//
// Copyright (c) 2016-2017 Federico Boniardi and Wolfram Burgard
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

#include "squirrel_2d_localizer/resampling.h"
#include "squirrel_2d_localizer/particle_types.h"

#include <algorithm>
#include <numeric>
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
  // Create new particles.
  Pose2d sampled_pose;
  for (int i = 0; i < nparticles_add; ++i) {
    sampled_pose.fromVector(
        cov * Eigen::Vector3d(randn(rg), randn(rg), randn(rg)) +
        mean.toVector());
    particles->emplace_back(sampled_pose, 0.);
  }
}

void uniformDownsample(
    int nparticles_remove, std::vector<Particle>* particles) {
  std::unique_lock<std::mutex> lock(__internal::resampling_mtx_);
  std::mt19937 rg(std::rand());
  const int new_particles_num = particles->size() - nparticles_remove;
  // Extract uniformly which particles to keep.
  std::vector<int> indices(particles->size());
  std::iota(indices.begin(), indices.end(), 0);
  std::shuffle(indices.begin(), indices.end(), rg);
  // Create the new particles set.
  std::vector<Particle> new_particles;
  new_particles.reserve(new_particles_num);
  for (int i = 0; i < new_particles_num; ++i)
    new_particles.emplace_back(particles->at(indices[i]));
  *particles = new_particles;
}

}  // namespace resampling

}  // namespace squirrel_2d_localizer
