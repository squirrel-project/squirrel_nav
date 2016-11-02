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

#include "squirrel_2d_localizer/resampling.h"

#include <random>
#include <thread>
#include <vector>

namespace squirrel_2d_localizer {

namespace resampling {

void importanceSampling(std::vector<Particle>* particles) {
  std::unique_lock<std::mutex> lock(internal::resampling_mtx_);
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

}  // namespace resampling

}  // namespace squirrel_2d_localizer
