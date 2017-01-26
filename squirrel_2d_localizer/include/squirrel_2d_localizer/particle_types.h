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

#ifndef SQUIRREL_2D_LOCALIZER_PARTICLE_TYPES_H_
#define SQUIRREL_2D_LOCALIZER_PARTICLE_TYPES_H_

#include "squirrel_2d_localizer/math_types.h"
#include "squirrel_2d_localizer/se2_types.h"

#include <angles/angles.h>

#include <cassert>
#include <cmath>
#include <vector>

namespace squirrel_2d_localizer {

struct Particle {
  Particle() : weight(0.){};
  Particle(const Pose2d& p, double w) : pose(p), weight(w) {}

  Pose2d pose;
  double weight;
};

namespace particles {

inline double computeTotalWeight(const std::vector<Particle>& particles) {
  double tot_weight = 0.;
  for (size_t i = 0; i < particles.size(); ++i)
    tot_weight += particles[i].weight;
  return tot_weight;
}

inline void setPoses(
    const std::vector<Pose2d> poses, std::vector<Particle>* particles) {
  assert(poses.size() == particles->size());
  for (size_t i           = 0; i < particles->size(); ++i)
    particles->at(i).pose = poses[i];
}

inline void setWeights(
    const std::vector<double>& weights, std::vector<Particle>* particles) {
  assert(weights.size() == particles->size());
  for (size_t i             = 0; i < particles->size(); ++i)
    particles->at(i).weight = weights[i];
}

inline void setWeight(double weight, std::vector<Particle>* particles) {
#pragma omp parallel for default(shared)
  for (size_t i             = 0; i < particles->size(); ++i)
    particles->at(i).weight = weight;
}

inline void computeMeanAndCovariance(
    const std::vector<Particle>& particles, Pose2d* mean,
    Matrix<3, 3>* covariance) {
  double mean_x = 0., mean_y = 0., mean_c = 0., mean_s = 0., sq_tot_w = 0.;
  for (size_t i = 0; i < particles.size(); ++i) {
    const double weight = particles[i].weight;
    mean_x += weight * particles[i].pose[0];
    mean_y += weight * particles[i].pose[1];
    mean_c += weight * std::cos(particles[i].pose[2]);
    mean_s += weight * std::sin(particles[i].pose[2]);
    sq_tot_w += weight * weight;
  }
  const double mean_a = std::atan2(mean_s, mean_c);
  *mean               = Pose2d(mean_x, mean_y, mean_a);
  const double unbias = 1 - sq_tot_w;
  double cov00 = 0., cov01 = 0., cov02 = 0., cov11 = 0., cov12 = 0., cov22 = 0.;
  for (size_t i = 0; i < particles.size(); ++i) {
    const double dx = particles[i].pose[0] - mean_x;
    const double dy = particles[i].pose[1] - mean_y;
    const double da = angles::normalize_angle(particles[i].pose[2] - mean_a);
    const double weight = particles[i].weight;
    cov00 += weight * dx * dx / unbias;
    cov01 += weight * dx * dy / unbias;
    cov02 += weight * dx * da / unbias;
    cov11 += weight * dy * dy / unbias;
    cov12 += weight * dy * da / unbias;
    cov22 += weight * da * da / unbias;
  }
  (*covariance)(0, 0) = cov00;
  (*covariance)(0, 1) = (*covariance)(1, 0) = cov01;
  (*covariance)(0, 2) = (*covariance)(2, 0) = cov02;
  (*covariance)(1, 1) = cov11;
  (*covariance)(1, 2) = (*covariance)(2, 1) = cov12;
  (*covariance)(2, 2) = cov22;
}

}  // namespace particles

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_PARTICLE_TYPES_H_ */
