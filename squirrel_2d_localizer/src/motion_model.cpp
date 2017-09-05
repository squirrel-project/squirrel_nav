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
