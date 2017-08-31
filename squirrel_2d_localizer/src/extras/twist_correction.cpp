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

#include "squirrel_2d_localizer/extras/twist_correction.h"

#include <angles/angles.h>

#include <cmath>

namespace squirrel_2d_localizer {

void TwistCorrection::reset() {
  corr_(0, 0) = params_.corr_xx;
  corr_(0, 1) = corr_(1, 0) = params_.corr_xy;
  corr_(0, 2) = corr_(2, 0) = params_.corr_xa;
  corr_(1, 1) = params_.corr_xy;
  corr_(1, 2) = corr_(2, 1) = params_.corr_ya;
  corr_(2, 2) = params_.corr_aa;
  corr_ *= params_.corr_magnitude;
  a_ = params_.alpha;
}

Pose2d TwistCorrection::correction(const Twist2d& twist) {
  if (!twist_) {
    twist_.reset(new Twist2d(twist));
    return Pose2d(0., 0., 0.);
  } else {
    applyAlphaFilter(twist);
  }
  const Twist2d& sq_twist          = thresholdSquaredMagnitude(*twist_);
  const Eigen::Vector3d correction = corr_ * sq_twist;
  return Pose2d(correction[0], correction[1], correction[2]);
}

void TwistCorrection::initialize() {
  twist_.reset(nullptr);
  corr_(0, 0) = params_.corr_xx;
  corr_(0, 1) = corr_(1, 0) = params_.corr_xy;
  corr_(0, 2) = corr_(2, 0) = params_.corr_xa;
  corr_(1, 1) = params_.corr_xy;
  corr_(1, 2) = corr_(2, 1) = params_.corr_ya;
  corr_(2, 2) = params_.corr_aa;
  corr_ *= params_.corr_magnitude;
  a_ = params_.alpha;
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

TwistCorrection::Params TwistCorrection::Params::defaultParams() {
  Params params;
  params.corr_magnitude = 1.;
  params.corr_xx        = 0.;
  params.corr_xy        = 0.;
  params.corr_xa        = 0.;
  params.corr_yy        = 0.;
  params.corr_ya        = 0.;
  params.corr_aa        = 1.;
  params.alpha          = 0.5;
  return params;
}

}  // namespace squirrel_2d_localizer
