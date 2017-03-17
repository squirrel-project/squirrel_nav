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

#include "squirrel_2d_localizer/extras/twist_correction.h"

#include <angles/angles.h>

#include <cmath>

namespace squirrel_2d_localizer {

TwistCorrection::TwistCorrection() : twist_(nullptr) {
  setDefaultParams();
  initialize();
}

TwistCorrection::TwistCorrection(const Params& params)
    : twist_(nullptr), params_(params) {
  initialize();
}

Pose2d TwistCorrection::correction(const Twist2d& twist) {
  if (!twist_) {
    twist_.reset(new Twist2d(twist));
    return Pose2d(0., 0., 0.);
  } else {
    applyAlphaFilter(twist);
  }
  const Twist2d& sq_twist    = thresholdSquaredMagnitude(*twist_);
  const Vector<3> correction = corr_ * sq_twist;
  return Pose2d(correction[0], correction[1], correction[2]);
}

void TwistCorrection::applyAlphaFilter(const Twist2d& twist) {
  if (twist.norm() < 0.01)
    *twist_ = Twist2d();
  else {
    (*twist_)[0] = a_ * (*twist_)[0] + (1 - a_) * twist[0];
    (*twist_)[1] = a_ * (*twist_)[1] + (1 - a_) * twist[1];
    (*twist_)[2] = a_ * (*twist_)[2] + (1 - a_) * twist[2];
  }
}

Twist2d TwistCorrection::thresholdSquaredMagnitude(const Twist2d& twist) const {
  const double sq_max_lin_vel = std::pow(params_.max_lin_vel, 2);
  const double sq_max_ang_vel = std::pow(params_.max_ang_vel, 2);
  const double sq_twist_x     = twist[0] * twist[0];
  const double sq_twist_y     = twist[1] * twist[1];
  const double sq_twist_a     = twist[2] * twist[2];
  Twist2d output;
  output[0] = std::copysign(std::min(sq_max_lin_vel, sq_twist_x), twist[0]);
  output[1] = std::copysign(std::min(sq_max_lin_vel, sq_twist_y), twist[1]);
  output[2] = std::copysign(std::min(sq_max_ang_vel, sq_twist_a), twist[2]);
  return output;
}

}  // namespace squirrel_2d_localizer
