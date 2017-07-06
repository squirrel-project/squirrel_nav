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

#include "squirrel_navigation/local_planner.h"

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <thread>

PLUGINLIB_DECLARE_CLASS(
    squirrel_navigation, LocalPlanner, squirrel_navigation::LocalPlanner,
    nav_core::BaseLocalPlanner)

namespace squirrel_navigation {

void LocalPlanner::initialize(
    std::string name, tf::TransformListener* tfl,
    costmap_2d::Costmap2DROS* costmap_ros) {
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<LocalPlannerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&LocalPlanner::reconfigureCallback, this, _1, _2));
  // Initlialize controller and motion planner.
  motion_planner_.initialize("~/" + name);
  cotroller_.initialize("~/" + name);
  // Initialize/reset internal observers.
  tfl_.reset(tfl);
  costmap_ros_.reset(costmap_ros);
  current_goal_.reset(nullptr);
  // Initialize publishers.
  ref_pub_  = pnh.advertise<visualization_msgs::Marker>("reference_pose", 1);
  traj_pub_ = pnh.advertise<geometry_msgs::PoseArray>("trajectory", 1);
  odom_sub_ = nh.subscribe(odom_topic_, 1, &LocalPlanner::odomCallback, this);
  // Print info.
  ROS_INFO_STREAM(
      "squirrel_navigation::LocalPlanner: initialization successful.");
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd) {
  std::unique_lock<std::mutex> lock(state_mtx_);
  // Compute the reference pose and perform safety check.
  geomeyty_msgs::Pose ref_pose;
  geometry_msgs::Twist ref_twist;
  motion_planner.computeReference(
      robot_pose_.header.stamp, &ref_pose, &ref_twist);
  if (math::linearDistance2D(robot_pose_.pose, ref_pose) >
          params_.max_safe_lin_displacement ||
      math::angularDistanceYaw(robot_pose_.pose, ref_pose) >
          params_.max_safe_ang_displacement) {
    ROS_WARN_STREAM(
        "squirrel_navigation::LocalPlanner: The robot is too far from the "
        "planned trajectory. Replanning requested.");
    return false;
  }
  // Compute the commands via PID controller in map frame.
  geometry_msgs::Twist map_cmd;
  controller_.computeCommand(
      robot_pose_.header.stamp, robot_pose_.pose, ref_pose, robot_twist_.twist,
      ref_twist, &map_cmd);
  // Transform the commands in robot frame.
  geometry_msgs::Twist robot_cmd;
  globalToRobotFrame(map_cmd, &robot_cmd);
  // Threshold the twist according to safety parameters.
  safeVelocityCommands(robot_cmd, &cmd);
  return true;
}

bool LocalPlanner::isGoalReached() {
  if (!current_goal_)
    return false;
  if (math::linearDistance2D(robot_pose_, *current_goal_) <=
          params_.goal_lin_tolerance &&
      math::angularDistanceYaw(robot_pose_, *current_goal_) <=
          params_.goal_ang_tolerance) {
    current_goal_.reset(nullptr);
    if (params_.verbose)
      ROS_INFO_STREAM("squirrel_navigation::LocalPlanner: Goal reached.");
    return true;
  }
  return false;
}

bool LocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& waypoints) {
  if (waypoints.empty())
    return false;
  if (!current_goal_) {
    goal_.reset(new geometry_msgs::Pose(waypoints.back().pose));
    controller_.reset();
    motion_planner_.reset(waypoints);
  } else {
    motion_planner_.update(waypoints);
  }
  publishTrajectory(ros::Time::now());
  return true;
}

void LocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  const std::string& map_frame_id = costmap_ros_->getFrameID();
  try {
    geometry_msgs::PoseStamped odom_robot_pose(odom->pose.pose, odom->header);
    tfl_->waitForTransform(
        map_frame_id, odom_frame_id, odom->header.stamp, ros::Duration(0.05));
    tfl_->transformPose(map_frame_id, odom_robot_pose, *robot_pose_);
    robot_twist_.twist = odom->pose.twist.twist;
  } catch (const tf::TransformException& ex) {
    ROS_ERROR("squirrel_navigation::LocalPlanner: " << ex.what());
  }
}

void LocalPlanner::reconfigureCallback(
    LocalPlannerConfig& config, uint32_t level) {
  params_.goal_lin_tolerance        = config.goal_lin_tolerance;
  params_.goal_ang_tolerance        = config.goal_ang_tolerance;
  params_.max_safe_lin_velocity     = config.max_safe_lin_velocity;
  params_.max_safe_ang_velocity     = config.max_safe_ang_velocity;
  params_.max_safe_lin_displacement = config.max_safe_lin_displacement;
  params_.max_safe_ang_displacement = config.max_safe_ang_displacement;
  params_.verbose                   = config.verbose;
}

void LocalPlanner::publishReference(
    const geometry_msgs::Pose& ref_pose, const ros::Time& stamp) const {
  visualization_msgs::Marker marker;
  marker.id              = 0;
  marker.header.stamp    = stamp;
  marker.header.frame_id = costmap_ros_->getGlobalFrameID();
  marker.ns              = "trajectory_reference";
  marker.type            = visualization_msgs::Marker::ARROW;
  marker.action          = visualization_msgs::Marker::MODIFY;
  marker.pose            = ref_pose;
  marker.scale.x         = 0.22;
  marker.scale.y         = 0.035;
  marker.scale.z         = 0.;
  marker.color.r         = 0.0;
  marker.color.g         = 1.0;
  marker.color.b         = 0.0;
  marker.color.a         = 0.5;
  traj_pub_.publish(marker);
}

void LocalPlanner::publishTrajectory(const ros::Time& stamp) const {
  const auto& waypoints = motion_planner.waypoints();
  // Create the trajectory message.
  geometry_msgs::PoseArray trajectory;
  trajectory.header.frame_id = costmap_ros_->getGlobalFrameID();
  trajectory.header.stamp    = stamp;
  trajectory.pose.reserve(waypoints.size());
  for (const auto& waypoint : waypoints)
    trajectory.pose.emplace_back(waypoint.pose);
  traj_pub_.publish(trajectory);
}

void LocalPlanner::robotToGlobalFrame(
    const geometry_msgs::Twist& robot_twist,
    geometry_msgs::Twist* map_twist) const {
  const double robot_yaw = tf::getYaw(robot_pose_);
  // Transform the twist.
  const double c = std::cos(robot_yaw), s = std::sin(robot_yaw);
  map_twist->linear.x  = c * robot_twist.linear.x - s * robot_twist.linear.y;
  map_twist->linear.y  = s * robot_twist.linear.x + s * robot_twist.linear.y;
  map_twist->angular.z = robot_twist.angular.z;
}

void LocalPlanner::globalToRobotFrame(
    const geometry_msgs::Twist& map_twist,
    geometry_msgs::Twist* robot_twist) const {
  const double robot_yaw = tf::getYaw(robot_pose_);
  // Transform the twist.
  const double c = std::cos(-robot_yaw), s = std::sin(-robot_yaw);
  robot_twist->linear.x  = c * map_twist.linear.x - s * map_twist.linear.y;
  robot_twist->linear.y  = s * map_twist.linear.y + c * map_twist.linear.y;
  robot_twist->angular.z = map_twist.angular.z
}

void LocalPlanner::safeVelocityCommands(
    const geometry_msgs::Twist& twist, geometry_msgs::Twist* safe_twist) const {
  *safe_twist = twist;
  // Rescaling the linear twist.
  const double twist_lin_magnitude = std::hypot(twist.linear.x, twist.linear.y);
  if (twist_lin_magnitude > params_.max_safe_lin_velocity) {
    safe_twist->linear.x =
        params_.max_safe_lin_velocity * twist.linear.x / twist_lin_magnitude;
    safe_twist->linear.y =
        params_.max_safe_lin_velocity * twist.linear.y / twist_lin_magnitude;
  }
  // Rescaling the angular twist.
  const double twist_ang_magnitude = std::abs(twist.angular.z);
  if (twist_ang_magnitude > params_.max_safe_ang_velocity)
    safe_twist->angular.z = params_.max_sage_ang_velocity;
}

MotionPlanner::Params MotionPlanner::Params::defaultParams() {
  Params params;
  params.goal_ang_tolerance        = 0.05;
  params.goal_lin_tolerance        = 0.05;
  params.max_safe_lin_velocity     = 0.5;
  params.max_safe_ang_velocity     = 0.7;
  params.max_safe_lin_displacement = 0.5;
  params.max_safe_ang_displacement = 1.0;
  params.verbose                   = false;
  return params;
}

}  // namespace squirrel_navigation
