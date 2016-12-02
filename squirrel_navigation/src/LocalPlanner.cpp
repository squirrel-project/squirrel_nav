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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "squirrel_navigation/LocalPlanner.h"

PLUGINLIB_DECLARE_CLASS(
    squirrel_navigation, LocalPlanner, squirrel_navigation::LocalPlanner,
    nav_core::BaseLocalPlanner)

namespace squirrel_navigation {

LocalPlanner::LocalPlanner(void)
    : dsrv_lp_(nullptr),
      dsrv_pid_(nullptr),
      controller_(nullptr),
      tf_(nullptr),
      trajectory_(nullptr),
      goal_(nullptr),
      delay_(0.0),
      max_linear_vel_(0.5),
      max_angular_vel_(0.7),
      xy_goal_tolerance_(0.05),
      yaw_goal_tolerance_(0.05),
      verbose_(false) {
  ROS_INFO("squirrel_navigation::LocalPlanner started");
}

LocalPlanner::~LocalPlanner(void) {
  TrajectoryPlanner::deleteTrajectory();

  if (goal_)
    delete goal_;

  if (controller_)
    delete controller_;

  if (dsrv_lp_)
    delete dsrv_lp_;

  if (dsrv_pid_)
    delete dsrv_pid_;

  odom_sub_.shutdown();
  traj_pub_.shutdown();
}

void LocalPlanner::initialize(
    std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
  name_ = name;

  tf_          = tf;
  costmap_ros_ = costmap_ros;
  trajectory_  = TrajectoryPlanner::getTrajectory();

  if (not controller_)
    controller_ = new ControllerPID;

  ros::NodeHandle pnh("~/" + name);
  if (not dsrv_lp_)
    dsrv_lp_ = new dynamic_reconfigure::Server<LocalPlannerPluginConfig>(pnh);
  dynamic_reconfigure::Server<LocalPlannerPluginConfig>::CallbackType pcb =
      boost::bind(&LocalPlanner::plannerReconfigureCallback, this, _1, _2);
  dsrv_lp_->setCallback(pcb);

  ros::NodeHandle pnh_c("~/" + name + "/controller");
  if (not dsrv_pid_)
    dsrv_pid_ =
        new dynamic_reconfigure::Server<ControllerPIDGainsConfig>(pnh_c);
  dynamic_reconfigure::Server<ControllerPIDGainsConfig>::CallbackType ccb =
      boost::bind(&LocalPlanner::controllerReconfigureCallback, this, _1, _2);
  dsrv_pid_->setCallback(ccb);

  ros::NodeHandle nh;
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
      odom_topic_, 1, &LocalPlanner::odometryCallback, this);
  traj_pub_ = pnh.advertise<visualization_msgs::Marker>("ref_pose", 10);
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  std::unique_lock<std::mutex> lock(guard_);

  tf::Stamped<tf::Pose> tf_robot_pose;
  costmap_ros_->getRobotPose(tf_robot_pose);

  const double t = odom_.header.stamp.toSec();

  const double odom_vx   = odom_.twist.twist.linear.x,
               odom_vy   = odom_.twist.twist.linear.y,
               odom_vyaw = odom_.twist.twist.angular.z;

  TrajectoryPlanner::Profile robot_pose, ref_pose;

  robot_pose.x   = tf_robot_pose.getOrigin().getX();
  robot_pose.y   = tf_robot_pose.getOrigin().getY();
  robot_pose.yaw = tf::getYaw(tf_robot_pose.getRotation());
  robot_pose.vx =
      std::cos(robot_pose.yaw) * odom_vx - std::sin(robot_pose.yaw) * odom_vy;
  robot_pose.vy =
      std::sin(robot_pose.yaw) * odom_vx + std::cos(robot_pose.yaw) * odom_vy;
  robot_pose.vyaw = odom_vyaw;

  ref_pose = trajectory_->getProfile(odom_.header.stamp);

  if (trajectory_->lostRobot(ref_pose, robot_pose)) {
    trajectory_->deactivate();
    controller_->deactivate();
    delete goal_;
    goal_ = nullptr;

    if (verbose_)
      ROS_WARN_STREAM(name_ << ": Lost trajectory, recomputing trajectory.");

    return false;
  }

  controller_->computeCommands(ref_pose, robot_pose, t, cmd_);
  normalizeCommands();

  cmd_vel.linear.x =
      std::cos(-robot_pose.yaw) * cmd_[0] - std::sin(-robot_pose.yaw) * cmd_[1];
  cmd_vel.linear.y =
      std::sin(-robot_pose.yaw) * cmd_[0] + std::cos(-robot_pose.yaw) * cmd_[1];
  cmd_vel.angular.z = cmd_[2];

  publishTrajectoryPose(ref_pose, odom_.header.stamp);

  return true;
}

bool LocalPlanner::isGoalReached(void) {
  tf::Stamped<tf::Pose> tf_robot_pose;
  costmap_ros_->getRobotPose(tf_robot_pose);

  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = tf_robot_pose.getOrigin().getX();
  robot_pose.position.y = tf_robot_pose.getOrigin().getY();
  robot_pose.orientation =
      tf::createQuaternionMsgFromYaw(tf::getYaw(tf_robot_pose.getRotation()));

  if (not goal_) {
    return false;
  } else if (
      linearDistance(goal_->position, robot_pose.position) <
          xy_goal_tolerance_ and
      angularDistance(goal_->orientation, robot_pose.orientation) <
          yaw_goal_tolerance_) {
    delete goal_;
    goal_ = nullptr;
    trajectory_->deactivate();
    controller_->deactivate();

    if (verbose_)
      ROS_INFO_STREAM(name_ << ": Goal reached.");

    return true;
  } else {
    return false;
  }
}

bool LocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (plan.size() < 1)
    return false;

  if (not goal_) {
    goal_ = new geometry_msgs::Pose;
    controller_->activate(ros::Time::now().toSec());
  }

  *goal_ = plan.back().pose;

  return true;
}

void LocalPlanner::controllerReconfigureCallback(
    ControllerPIDGainsConfig& config, uint32_t level) {
  ControllerPID::Gain K;
  K.Pxy  = config.P_linear;
  K.Pyaw = config.P_angular;
  K.Ixy  = config.I_linear;
  K.Iyaw = config.I_angular;
  K.Dxy  = config.D_linear;
  K.Dyaw = config.D_angular;

  delay_ = config.delay;

  if (controller_)
    controller_->setGains(K);
  else
    ROS_ERROR_STREAM(
        name_ << ": Unable to reconfigure the controller parameters.");
}

void LocalPlanner::plannerReconfigureCallback(
    LocalPlannerPluginConfig& config, uint32_t level) {
  verbose_ = config.verbose;
  if (verbose_)
    ROS_INFO_STREAM(name_ << ": Set verbose.");

  odom_topic_ = config.odom_topic;
  if (verbose_)
    ROS_INFO_STREAM(name_ << ": Subscribing to " << odom_topic_);

  xy_goal_tolerance_  = config.xy_goal_tolerance;
  yaw_goal_tolerance_ = config.yaw_goal_tolerance;
  max_linear_vel_     = config.max_linear_vel;
  max_angular_vel_    = config.max_angular_vel;

  if (max_linear_vel_ <= 0) {
    if (verbose_)
      ROS_WARN_STREAM(name_ << ": max_linear_vel has been chosen to be non "
                               "positive. Reverting to 0.3 (m/s)");
    max_linear_vel_ = 0.5;
  }

  if (max_angular_vel_ <= 0) {
    if (verbose_)
      ROS_WARN_STREAM(name_ << ": max_angular_vel has been chosen to be non "
                               "positive. Reverting to 0.5 (rad/s)");
    max_angular_vel_ = 0.7;
  }

  if (trajectory_)
    trajectory_->setVelocityBounds(max_linear_vel_, max_angular_vel_);
  else
    ROS_WARN_STREAM(name_ << ": Unable to set velocity bounds to "
                             "squirrel_navigation::TrajectoryPlanner.");
}

void LocalPlanner::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  odom_ = *odom;
}

}  // namespace squirrel_navigation
