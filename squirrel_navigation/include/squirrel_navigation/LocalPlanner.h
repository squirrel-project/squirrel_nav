// Copyright (c) 2016, Federico Boniardi
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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//

#ifndef SQUIRREL_NAVIGATION_LOCALPLANNER_H_
#define SQUIRREL_NAVIGATION_LOCALPLANNER_H_

#include <ros/ros.h>

#include <nav_core/base_local_planner.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <tf/tf.h>

#include <angles/angles.h>

#include <pluginlib/class_list_macros.h>

#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "squirrel_navigation/Common.h"
#include "squirrel_navigation/ControllerPID.h"
#include "squirrel_navigation/ControllerPIDGainsConfig.h"
#include "squirrel_navigation/LocalPlannerPluginConfig.h"
#include "squirrel_navigation/TrajectoryPlanner.h"

#include <string>
#include <thread>
#include <vector>

namespace squirrel_navigation {

class LocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  LocalPlanner(void);
  virtual ~LocalPlanner(void);

  void initialize(
      std::string, tf::TransformListener*, costmap_2d::Costmap2DROS*);

  bool computeVelocityCommands(geometry_msgs::Twist&);
  bool isGoalReached(void);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>&);

 private:
  ControllerPID* controller_;
  TrajectoryPlanner* trajectory_;
  tf::TransformListener* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  ros::Subscriber odom_sub_;
  ros::Publisher traj_pub_;

  std::mutex guard_;

  double cmd_[3];

  nav_msgs::Odometry odom_;

  geometry_msgs::Pose* goal_;

  // Parameters
  dynamic_reconfigure::Server<LocalPlannerPluginConfig>* dsrv_lp_;
  dynamic_reconfigure::Server<ControllerPIDGainsConfig>* dsrv_pid_;
  ControllerPID::Gain gains_;
  double max_linear_vel_, max_angular_vel_;
  double yaw_goal_tolerance_, xy_goal_tolerance_;
  double delay_;
  bool verbose_;
  std::string odom_topic_;

  std::string name_;

  void controllerReconfigureCallback(ControllerPIDGainsConfig&, uint32_t);
  void plannerReconfigureCallback(LocalPlannerPluginConfig&, uint32_t);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr&);

  inline void normalizeCommands(void) {
    const double lin_norm = std::hypot(cmd_[0], cmd_[1]);
    const double ang_norm = std::abs(cmd_[2]);
    const double lin_rescaler =
        lin_norm > max_linear_vel_ ? max_linear_vel_ / lin_norm : 1.0;
    const double ang_rescaler =
        ang_norm > max_angular_vel_ ? max_angular_vel_ / ang_norm : 1.0;

    cmd_[0] *= lin_rescaler;
    cmd_[1] *= lin_rescaler;
    cmd_[2] *= ang_rescaler;
  };

  inline void publishTrajectoryPose(
      const TrajectoryPlanner::Profile& p, const ros::Time& t) {
    visualization_msgs::Marker marker;
    marker.id              = 0;
    marker.header.stamp    = t;
    marker.header.frame_id = "/map";
    marker.ns              = "waypoints";
    marker.type            = visualization_msgs::Marker::ARROW;

    marker.action           = visualization_msgs::Marker::MODIFY;
    marker.pose.position.x  = p.x;
    marker.pose.position.y  = p.y;
    marker.pose.position.z  = 0.0;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(p.yaw);
    marker.scale.x          = 0.22;
    marker.scale.y          = 0.035;
    marker.scale.z          = 0.0;
    marker.color.a          = 0.5;
    marker.color.r          = 0.0;
    marker.color.g          = 1.0;
    marker.color.b          = 0.0;

    traj_pub_.publish(marker);
  };
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_LOCALPLANNER_H_ */
