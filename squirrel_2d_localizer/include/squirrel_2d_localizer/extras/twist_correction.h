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

  inline Params& params() { return params_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
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
