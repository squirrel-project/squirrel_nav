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

#ifndef SQUIRREL_2D_LOCALIZATION_TWIST_CORRECTION_H_
#define SQUIRREL_2D_LOCALIZATION_TWIST_CORRECTION_H_

#include "squirrel_2d_localizer/se2_types.h"
#include "squirrel_2d_localizer/extras/twist_types.h"

#include <cmath>
#include <memory>

namespace squirrel_2d_localizer {

class TwistCorrection {
 public:
  typedef std::unique_ptr<TwistCorrection> Ptr;
  typedef std::unique_ptr<TwistCorrection const> ConstPtr;

  struct Params {
    double corr_xx, corr_xy, corr_xa, corr_yy, corr_ya, corr_aa;
    double corr_magnitude;
    double max_lin_vel, max_ang_vel;
    double alpha;
  };

 public:
  TwistCorrection();
  TwistCorrection(const Params& params);
  virtual ~TwistCorrection() {}
  
  Pose2d correction(const Twist2d& twist);

  Params& params() { return params_; }

 private:
  inline void setDefaultParams() {
    params_.corr_magnitude = 1.;
    params_.corr_xx        = 0.;
    params_.corr_xy        = 0.;
    params_.corr_xa        = 0.;
    params_.corr_yy        = 0.;
    params_.corr_ya        = 0.;
    params_.corr_aa        = 1.;
    params_.alpha          = 0.5;
  }

  inline void initialize() {
    corr_(0, 0) = params_.corr_xx;
    corr_(0, 1) = corr_(1, 0) = params_.corr_xy;
    corr_(0, 2) = corr_(2, 0) = params_.corr_xa;
    corr_(1, 1) = params_.corr_xy;
    corr_(1, 2) = corr_(2, 1) = params_.corr_ya;
    corr_(2, 2) = params_.corr_aa;
    corr_ *= params_.corr_magnitude;
    a_ = params_.alpha;
  }

 private:
  void applyAlphaFilter(const Twist2d& twist);
  Twist2d thresholdSquaredMagnitude(const Twist2d& twist) const;
  
 private:
  Matrix<3, 3> corr_;
  double a_;

  std::unique_ptr<Twist2d> twist_;
  
  Params params_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZATION_TWIST_CORRECTION_H_ */
