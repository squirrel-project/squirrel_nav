// The MIT License (MIT)
//
// Copyright (c) 2017 Federico Boniardi and Wolfram Burgard
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
