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

  mutable std::mutex update_mtx_, reset_mtx_, tf_mtx_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LOCALIZER_ROS_H_ */
