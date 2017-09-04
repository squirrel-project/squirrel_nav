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

#include "squirrel_2d_localizer/motion_model.h"

#include <angles/angles.h>

#include <thread>

namespace squirrel_2d_localizer {

void MotionModel::sampleProposal(
    const Transform2d& motion, std::vector<Particle>* particles) const {
  std::unique_lock<std::mutex> lock(mtx_);
  std::normal_distribution<double> randn(0., 1.);
  const double dx = motion[0], dy = motion[1],
               da = angles::normalize_angle(motion[2]);
  const double mx = params_.noise_magnitude * std::abs(dx),
               my = params_.noise_magnitude * std::abs(dy),
               ma = params_.noise_magnitude * std::abs(da);
  const double sx = params_.noise_xx * mx +
                    params_.noise_xy * my + params_.noise_xa * ma;
  const double sy = params_.noise_xy * mx +
                    params_.noise_yy * my + params_.noise_ya * ma;
  const double sa = params_.noise_xa * mx +
                    params_.noise_ya * my + params_.noise_aa * ma;
  for (size_t i = 0; i < particles->size(); ++i) {
    const Pose2d e(
        dx + sx * randn(rnd_eng_), dy + sy * randn(rnd_eng_),
        da + sa * randn(rnd_eng_));
    particles->at(i).pose *= e;
  }
}

MotionModel::Params MotionModel::Params::defaultParams() {
  Params params;
  params.noise_xx        = 1.;
  params.noise_xy        = 0.;
  params.noise_xa        = 0.;
  params.noise_yy        = 1.;
  params.noise_ya         = 0.;
  params.noise_aa        = 1.;
  params.noise_magnitude = 0.5;
  return params;
}

}  // namespace squirrel_2d_localizer
