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
