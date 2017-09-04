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

#include "squirrel_2d_localizer/extras/twist_types.h"
#include "squirrel_2d_localizer/se2_types.h"

#include <memory>
#include <cmath>

namespace squirrel_2d_localizer {

class TwistCorrection {
 public:
  struct Params {
    static Params defaultParams();
    
    double corr_xx, corr_xy, corr_xa, corr_yy, corr_ya, corr_aa;
    double corr_magnitude;
    double max_lin_vel, max_ang_vel;
    double alpha;
  };

 public:
  TwistCorrection() : params_(Params::defaultParams()) { initialize(); }
  TwistCorrection(const Params& params) : params_(params) { initialize(); }
  virtual ~TwistCorrection() {}

  void reset();

  // Compute the correction twist.
  Pose2d correction(const Twist2d& twist);

  // Paramters read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  // Initialize.
  void initialize();

  // Filter and threshold the final value.
  void applyAlphaFilter(const Twist2d& twist);
  Twist2d thresholdSquaredMagnitude(const Twist2d& twist) const;

 private:
  Eigen::Matrix3d corr_;
  double a_;

  std::unique_ptr<Twist2d> twist_;

  Params params_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZATION_TWIST_CORRECTION_H_ */
