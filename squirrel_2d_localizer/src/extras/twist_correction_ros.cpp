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

#include "squirrel_2d_localizer/extras/twist_correction_ros.h"
#include "squirrel_2d_localizer/se2_types.h"

#include <geometry_msgs/Twist.h>

namespace squirrel_2d_localizer {

TwistCorrectionROS::TwistCorrectionROS()
    : odom_sub_(nh_, "/odom", 1), cache_(odom_sub_, 100) {
  ros::NodeHandle pnh("~/twist_correction");
  TwistCorrection::Params params;
  pnh.param<double>("corr_xx", params.corr_xx, 0.);
  pnh.param<double>("corr_xy", params.corr_xy, 0.);
  pnh.param<double>("corr_xa", params.corr_xa, 0.);
  pnh.param<double>("corr_yy", params.corr_yy, 0.);
  pnh.param<double>("corr_ya", params.corr_ya, 0.);
  pnh.param<double>("corr_aa", params.corr_aa, 1.);
  pnh.param<double>("corr_magnitude", params.corr_magnitude, 1.);
  pnh.param<double>("alpha_filter", params.alpha, 0.5);
  pnh.param<double>("max_lin_vel", params.max_lin_vel, 0.5);
  pnh.param<double>("max_ang_vel", params.max_ang_vel, 0.7);
  twist_correction_.reset(new TwistCorrection(params));
}

Pose2d TwistCorrectionROS::correction(const ros::Time& time) const {
  const auto& odom_before_time = cache_.getElemBeforeTime(time);
  const auto& odom_after_time  = cache_.getElemAfterTime(time);
  if (!odom_before_time || !odom_after_time)
    return Pose2d(0., 0., 0.);
  const geometry_msgs::Twist& tw0 = odom_before_time->twist.twist;
  const geometry_msgs::Twist& tw1 = odom_after_time->twist.twist;
  const double t0 = odom_before_time->header.stamp.toSec();
  const double t1 = odom_after_time->header.stamp.toSec();
  // interpolate the twist
  const double dt = t1 - t0;
  const double t  = time.toSec() - t0;
  const Twist2d twist(
      linearInterpolation(tw0.linear.x, tw1.linear.x, t, dt),
      linearInterpolation(tw0.linear.y, tw1.linear.y, t, dt),
      linearInterpolation(tw0.angular.z, tw1.angular.z, t, dt));
  return twist_correction_->correction(twist);
}

}  // namespace squirrel_2d_localizer
