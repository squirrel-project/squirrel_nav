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
#include "squirrel_navigation/safety/arm_skin_observer.h"
#include "squirrel_navigation/safety/scan_observer.h"
#include "squirrel_navigation/utils/math_utils.h"

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
  if (init_)
    return;
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<LocalPlannerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&LocalPlanner::reconfigureCallback, this, _1, _2));
  // Initlialize controller and motion planner, by now only PID and linear.
  controller_.reset(new ControllerPID);
  controller_->initialize(name + "/ControllerPID");
  motion_planner_.reset(new LinearMotionPlanner);
  motion_planner_->initialize(name + "/MotionPlanner");
  // Initialize/reset internal observers.
  tfl_.reset(tfl);
  costmap_ros_.reset(costmap_ros);
  current_goal_.reset(nullptr);
  // Initialize the safety observers.
  if (pnh.hasParam("safety_observers")) {
    pnh.getParam("safety_observers", params_.safety_observers);
    for (const auto& safety_observer_tag : params_.safety_observers) {
      // Scan observer.
      if (safety_observer_tag == "ScanObserver") {
        safety::ScanObserver* scan_observer = new safety::ScanObserver;
        safety_observers_.emplace_back(scan_observer);
        scan_observer->initialize(name + "/ScanObserver");
      }
      if (safety_observer_tag == "ArmSkinObserver") {
        safety::ArmSkinObserver* arm_observer = new safety::ArmSkinObserver;
        safety_observers_.emplace_back(arm_observer);
        arm_observer->initialize(name + "/ArmSkinObserver");
      }
    }
  }
  // Initialize publishers.
  robot_pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("debug", 1);
  ref_pub_  = pnh.advertise<visualization_msgs::Marker>("reference_pose", 1);
  traj_pub_ = pnh.advertise<geometry_msgs::PoseArray>("trajectory", 1);
  odom_sub_ =
      nh.subscribe(params_.odom_topic, 1, &LocalPlanner::odomCallback, this);
  // Initialization successful.
  init_ = true;
  ROS_INFO_STREAM(
      "squirrel_navigation::LocalPlanner: initialization successful.");
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd) {
  std::unique_lock<std::mutex> lock(state_mtx_);
  // Check the scan observer.
  for (const auto& safety_observer : safety_observers_)
    if (!safety_observer->safe()) {
      cmd.linear.x  = 0.0;
      cmd.linear.y  = 0.0;
      cmd.angular.z = 0.0;
      return true;
    }
  // Compute the reference pose and perform safety check.
  const ros::Time& stamp = robot_pose_.header.stamp;
  geometry_msgs::Pose ref_pose;
  geometry_msgs::Twist ref_twist;
  motion_planner_->computeReference(stamp, &ref_pose, &ref_twist);
  publishReference(ref_pose, stamp);
  robot_pose_pub_.publish(robot_pose_);
  if (math::linearDistance2D(robot_pose_.pose, ref_pose) >
          params_.max_safe_lin_displacement ||
      math::angularDistanceYaw(robot_pose_.pose, ref_pose) >
          params_.max_safe_ang_displacement) {

    std::cout << robot_pose_.pose << std::endl;
    std::cout << ref_pose << std::endl;

    ROS_WARN_STREAM(
        "squirrel_navigation::LocalPlanner: The robot is too far from the "
        "planned trajectory. Replanning requested.");
    current_goal_.reset(nullptr);
    return false;
  }
  // Compute the commands via PID controller in map frame.
  geometry_msgs::Twist map_cmd;
  controller_->computeCommand(
      stamp, robot_pose_.pose, ref_pose, robot_twist_.twist, ref_twist,
      &map_cmd);
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
  if (math::linearDistance2D(robot_pose_.pose, *current_goal_) <=
          params_.goal_lin_tolerance &&
      math::angularDistanceYaw(robot_pose_.pose, *current_goal_) <=
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
  const ros::Time& stamp = robot_pose_.header.stamp;
  if (newGoal(waypoints.back().pose)) {
    current_goal_.reset(new geometry_msgs::Pose(waypoints.back().pose));
    controller_->reset(stamp);
    motion_planner_->reset(waypoints, stamp);
  } else {
    motion_planner_->update(waypoints, stamp);
  }
  publishTrajectory(stamp);
  return true;
}

void LocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  ROS_INFO_STREAM_ONCE(
      "squirrel_localizer::LocalPlanner: Subscribed to odometry.");
  // Update the internal state.
  std::unique_lock<std::mutex> lock(state_mtx_);
  const std::string& map_frame_id = costmap_ros_->getGlobalFrameID();
  try {
    geometry_msgs::PoseStamped odom_robot_pose;
    odom_robot_pose.header = odom->header;
    odom_robot_pose.pose   = odom->pose.pose;
    tfl_->waitForTransform(
        map_frame_id, odom->header.frame_id, odom->header.stamp,
        ros::Duration(0.1));
    tfl_->transformPose(map_frame_id, odom_robot_pose, robot_pose_);
    robotToGlobalFrame(odom->twist.twist, &robot_twist_.twist);
  } catch (const tf::TransformException& ex) {
    ROS_ERROR_STREAM("squirrel_navigation::LocalPlanner: " << ex.what());
  }
}

void LocalPlanner::reconfigureCallback(
    LocalPlannerConfig& config, uint32_t level) {
  params_.odom_topic                = config.odom_topic;
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
  marker.ns              = "reference";
  marker.type            = visualization_msgs::Marker::ARROW;
  marker.action          = visualization_msgs::Marker::MODIFY;
  marker.pose            = ref_pose;
  marker.scale.x         = 0.22;
  marker.scale.y         = 0.035;
  marker.scale.z         = 0.05;
  marker.color.r         = 0.0;
  marker.color.g         = 1.0;
  marker.color.b         = 0.0;
  marker.color.a         = 0.5;
  ref_pub_.publish(marker);
}

void LocalPlanner::publishTrajectory(const ros::Time& stamp) const {
  const auto& waypoints =
      static_cast<const LinearMotionPlanner*>(motion_planner_.get())
          ->waypoints();
  // Create the trajectory message.
  geometry_msgs::PoseArray trajectory;
  trajectory.header.frame_id = costmap_ros_->getGlobalFrameID();
  trajectory.header.stamp    = stamp;
  trajectory.poses.reserve(waypoints.size());
  for (const auto& waypoint : waypoints)
    trajectory.poses.emplace_back(waypoint.pose);
  traj_pub_.publish(trajectory);
}

void LocalPlanner::robotToGlobalFrame(
    const geometry_msgs::Twist& robot_twist,
    geometry_msgs::Twist* map_twist) const {
  const double robot_yaw = tf::getYaw(robot_pose_.pose.orientation);
  // Transform the twist.
  const double c = std::cos(robot_yaw), s = std::sin(robot_yaw);
  map_twist->linear.x  = c * robot_twist.linear.x - s * robot_twist.linear.y;
  map_twist->linear.y  = s * robot_twist.linear.x + s * robot_twist.linear.y;
  map_twist->angular.z = robot_twist.angular.z;
}

void LocalPlanner::globalToRobotFrame(
    const geometry_msgs::Twist& map_twist,
    geometry_msgs::Twist* robot_twist) const {
  const double robot_yaw = tf::getYaw(robot_pose_.pose.orientation);
  // Transform the twist.
  const double c = std::cos(-robot_yaw), s = std::sin(-robot_yaw);
  robot_twist->linear.x  = c * map_twist.linear.x - s * map_twist.linear.y;
  robot_twist->linear.y  = s * map_twist.linear.y + c * map_twist.linear.y;
  robot_twist->angular.z = map_twist.angular.z;
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
    safe_twist->angular.z = params_.max_safe_ang_velocity;
}

bool LocalPlanner::newGoal(const geometry_msgs::Pose& pose) const {
  if (!current_goal_)
    return true;
  bool new_position    = math::linearDistance2D(*current_goal_, pose) > 1e-8;
  bool new_orientation = math::angularDistanceYaw(*current_goal_, pose) > 1e-8;
  return new_position || new_orientation;
}

LocalPlanner::Params LocalPlanner::Params::defaultParams() {
  Params params;
  params.odom_topic                = "/odom";
  params.goal_ang_tolerance        = 0.05;
  params.goal_lin_tolerance        = 0.05;
  params.max_safe_lin_velocity     = 0.5;
  params.max_safe_ang_velocity     = 0.7;
  params.max_safe_lin_displacement = 0.5;
  params.max_safe_ang_displacement = 1.0;
  params.safety_observers          = {"SafetyScanObserver"};
  params.verbose                   = false;
  return params;
}

}  // namespace squirrel_navigation
