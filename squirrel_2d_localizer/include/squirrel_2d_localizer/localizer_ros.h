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

#ifndef SQUIRREL_2D_LOCALIZER_LOCALIZER_ROS_H_
#define SQUIRREL_2D_LOCALIZER_LOCALIZER_ROS_H_

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include "squirrel_2d_localizer/localizer.h"
#include "squirrel_2d_localizer/extras/twist_correction_ros.h"

#include <mutex>

namespace squirrel_2d_localizer {

class LocalizerROS {
 public:
  LocalizerROS();
  virtual ~LocalizerROS();

  void spin(double hz = 40.);

 public:
  mutable std::mutex update_mtx;
  
 private:
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  bool lookupOdometry(
      const ros::Time& stamp, const ros::Duration& timeout,
      tf::StampedTransform* tf_o2r);

  void publishTransform(const ros::Time& stamp);
  void publishParticles(const ros::Time& stamp);
  void publishPoseWithCovariance(const ros::Time& stamp);

 private:
  Localizer::Ptr localizer_;

  tf::StampedTransform tf_m2r_, tf_o2r_;
  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;

  ros::Publisher pose_pub_, particles_pub_;
  ros::Subscriber scan_sub_, initpose_sub_;

  bool use_last_pose_;
  double init_x_, init_y_, init_a_;

  bool use_twist_correction_;
  TwistCorrectionROS::Ptr twist_correction_;
  
  std::string map_frame_id_, odom_frame_id_, robot_frame_id_;
  std::string node_name_;

  bool update_laser_params_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LOCALIZER_ROS_H_ */
