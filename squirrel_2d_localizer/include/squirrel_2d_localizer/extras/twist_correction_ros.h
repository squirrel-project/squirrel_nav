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

#ifndef SQUIRREL_2D_LOCALIZER_TWIST_CORRECTION_ROS_H_
#define SQUIRREL_2D_LOCALIZER_TWIST_CORRECTION_ROS_H_

#include "squirrel_2d_localizer/extras/twist_correction.h"

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include <memory>

namespace squirrel_2d_localizer {

class TwistCorrectionROS {
 public:
  typedef std::unique_ptr<TwistCorrectionROS> Ptr;
  typedef std::unique_ptr<TwistCorrectionROS const> ConstPtr;

 public:
  TwistCorrectionROS();
  virtual ~TwistCorrectionROS() {}

  Pose2d correction(const ros::Time& time) const;

 private:
  inline double linearInterpolation(
      double x0, double x1, double dt, double t) const {
    return (1 - t / dt) * x0 + (t / dt) * x1;
  }

 private:
  TwistCorrection::Ptr twist_correction_;

  ros::NodeHandle nh_;

  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Cache<nav_msgs::Odometry> cache_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_TWIST_CORRECTION_ROS_H_ */
