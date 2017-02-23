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

MotionModel::MotionModel(const Params& motion_params)
    : rnd_eng_(std::rand()), motion_params_(motion_params) {}

void MotionModel::sampleProposal(
    const Transform2d& motion, std::vector<Particle>* particles) const {
  std::unique_lock<std::mutex> lock(mtx_);
  std::normal_distribution<double> randn(0., 1.);
  const double dx = motion[0], dy = motion[1],
               da = angles::normalize_angle(motion[2]);
  const double mx = motion_params_.noise_magnitude * std::abs(dx),
               my = motion_params_.noise_magnitude * std::abs(dy),
               ma = motion_params_.noise_magnitude * std::abs(da);
  const double sx = motion_params_.noise_xx * mx +
                    motion_params_.noise_xy * my + motion_params_.noise_xa * ma;
  const double sy = motion_params_.noise_xy * mx +
                    motion_params_.noise_yy * my + motion_params_.noise_ya * ma;
  const double sa = motion_params_.noise_xa * mx +
                    motion_params_.noise_ya * my + motion_params_.noise_aa * ma;
  for (size_t i = 0; i < particles->size(); ++i) {
    const Pose2d e(
        dx + sx * randn(rnd_eng_), dy + sy * randn(rnd_eng_),
        da + sa * randn(rnd_eng_));
    particles->at(i).pose *= e;
  }
}

}  // namespace squirrel_2d_localizer
