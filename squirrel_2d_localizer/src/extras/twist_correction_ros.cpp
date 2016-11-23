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
