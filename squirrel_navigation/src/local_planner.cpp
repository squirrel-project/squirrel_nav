// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the University of Freiburg nor the names of
//   its contributors may be used to endorse or promote products
//   derived from this software without specific prior written
//   permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include "squirrel_navigation/local_planner.h"
#include "squirrel_navigation/safety/arm_skin_observer.h"
#include "squirrel_navigation/safety/scan_observer.h"
#include "squirrel_navigation/utils/footprint_utils.h"
#include "squirrel_navigation/utils/math_utils.h"

#include <pluginlib/class_list_macros.h>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>

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
  ros::NodeHandle pnh("~/" + name), mbnh("~"), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<LocalPlannerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&LocalPlanner::reconfigureCallback, this, _1, _2));
  // Initlialize controller and motion planner, by now only PID and linear.
  controller_.reset(new ControllerPID);
  controller_->initialize(name + "/ControllerPID");
  motion_planner_.reset(new LinearMotionPlanner);
  motion_planner_->initialize(name + "/LinearMotionPlanner");
  // Initialize/reset internal observers.
  tfl_.reset(tfl);
  costmap_ros_.reset(costmap_ros);
  current_goal_.reset(nullptr);
  // Initialize the safety observers.
  if (pnh.hasParam("safety_observers")) {
    pnh.getParam("safety_observers", params_.safety_observers);
    for (const auto& safety_observer_tag : params_.safety_observers) {
      // Scan observer.
      if (std::string(safety_observer_tag) == safety::ScanObserver::tag) {
        safety::ScanObserver* scan_observer = new safety::ScanObserver;
        safety_observers_.emplace_back(scan_observer);
        scan_observer->initialize(name + "/" + safety::ScanObserver::tag);
      }
      if (std::string(safety_observer_tag) == safety::ArmSkinObserver::tag) {
        safety::ArmSkinObserver* arm_observer = new safety::ArmSkinObserver;
        safety_observers_.emplace_back(arm_observer);
        arm_observer->initialize(name + "/" + safety::ArmSkinObserver::tag);
      }
    }
  }
  // Initialize the collision detector and replanning stamp.
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  costmap_model_.reset(new base_local_planner::CostmapModel(*costmap));
  // Initialize publishers, subscriber and services.
  cmd_pub_ =
      pnh.advertise<visualization_msgs::MarkerArray>("cmd_navigation", 1);
  ref_pub_  = pnh.advertise<visualization_msgs::Marker>("reference_pose", 1);
  traj_pub_ = pnh.advertise<geometry_msgs::PoseArray>("trajectory", 1);
  footprints_pub_ =
      pnh.advertise<visualization_msgs::MarkerArray>("footprints", 1);
  odom_sub_ =
      nh.subscribe(params_.odom_topic, 1, &LocalPlanner::odomCallback, this);
  footprint_sub_ = nh.subscribe(
      params_.footprint_topic, 1, &LocalPlanner::footprintCallback, this);
  brake_srv_ = pnh.advertiseService(
      "brakeRobot", &LocalPlanner::brakeRobotCallback, this);
  unbrake_srv_ = pnh.advertiseService(
      "unbrakeRobot", &LocalPlanner::unbrakeRobotCallback, this);
  // Initialization successful.
  last_nwaypoints_ = -1;
  init_            = true;
  ROS_INFO_STREAM(
      "squirrel_navigation/LocalPlanner: initialization successful.");
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd) {
  std::unique_lock<std::mutex> lock(state_mtx_);
  // Check if the robot is braked.
  if (base_brake_.spin(&cmd))
    return true;
  // Check the scan observer.
  for (const auto& safety_observer : safety_observers_)
    if (!safety_observer->safe())
      return true;
  // Compute the reference pose and perform safety check.
  const ros::Time& stamp = robot_pose_.header.stamp;
  geometry_msgs::Pose ref_pose;
  geometry_msgs::Twist ref_twist;
  std::vector<geometry_msgs::PoseStamped> trajectory;
  motion_planner_->computeReference(stamp, &ref_pose, &ref_twist);
  publishReference(ref_pose, stamp);
  if (math::linearDistance2D(robot_pose_.pose, ref_pose) >
          params_.max_safe_lin_displacement ||
      math::angularDistanceYaw(robot_pose_.pose, ref_pose) >
          params_.max_safe_ang_displacement) {
    ROS_WARN_STREAM(
        "squirrel_navigation/LocalPlanner: The robot is too far from the "
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
  twistToRobotFrame(map_cmd, &robot_cmd);
  // Threshold the twist according to safety parameters.
  safeVelocityCommands(robot_cmd, &cmd);
  // Publish the command.
  publishTwist(robot_pose_, cmd);
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
      ROS_INFO_STREAM("squirrel_navigation/LocalPlanner: Goal reached.");
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
    publishTrajectory(stamp);
    publishFootprints(stamp);
  } else if (
      !params_.collision_based_replanning ||
      needReplanning(motion_planner_->trajectory(), waypoints)) {
    motion_planner_->update(waypoints, stamp);
    publishTrajectory(stamp);
    publishFootprints(stamp);
  }
  return true;
}

void LocalPlanner::footprintCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(
      "squirrel_navigation/LocalPlanner: Subscribed to the footprint.");
  bool footprint_changed = false;
  if (footprint_.size() == msg->polygon.points.size()) {
    for (unsigned int i = 0; i < footprint_.size(); ++i)
      if (math::linearDistance2D(footprint_[i], msg->polygon.points[i]) >
          0.01) {
        footprint_changed = true;
        break;
      }
  } else {
    footprint_changed = true;
  }
  // If footprint changed update the internal values.
  if (footprint_changed) {
    footprint_ = costmap_2d::toPointVector(msg->polygon);
    costmap_2d::calculateMinAndMaxDistances(
        footprint_, inscribed_radius_, circumscribed_radius_);
  }
}

void LocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  ROS_INFO_STREAM_ONCE(
      "squirrel_localizer/LocalPlanner: Subscribed to odometry.");
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
    twistToGlobalFrame(odom->twist.twist, &robot_twist_.twist);
  } catch (const tf::TransformException& ex) {
    ROS_ERROR_STREAM("squirrel_navigation/LocalPlanner: " << ex.what());
  }
}

void LocalPlanner::reconfigureCallback(
    LocalPlannerConfig& config, uint32_t level) {
  params_.odom_topic                   = config.odom_topic;
  params_.footprint_topic              = config.footprint_topic;
  params_.collision_based_replanning   = config.collision_based_replanning;
  params_.replanning_lin_lookahead     = config.replanning_lin_lookahead;
  params_.replanning_ang_lookahead     = config.replanning_ang_lookahead;
  params_.replanning_path_length_ratio = config.replanning_path_length_ratio;
  params_.goal_lin_tolerance           = config.goal_lin_tolerance;
  params_.goal_ang_tolerance           = config.goal_ang_tolerance;
  params_.max_safe_lin_velocity        = config.max_safe_lin_velocity;
  params_.max_safe_ang_velocity        = config.max_safe_ang_velocity;
  params_.max_safe_lin_displacement    = config.max_safe_lin_displacement;
  params_.max_safe_ang_displacement    = config.max_safe_ang_displacement;
  params_.visualize_topics             = config.visualize_topics;
  params_.verbose                      = config.verbose;
}

bool LocalPlanner::brakeRobotCallback(
    squirrel_navigation_msgs::BrakeRobot::Request& req,
    squirrel_navigation_msgs::BrakeRobot::Response& res) {
  base_brake_.enable(req.seconds);
  return true;
}

bool LocalPlanner::unbrakeRobotCallback(
    std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  base_brake_.disable();
  return true;
}

void LocalPlanner::publishReference(
    const geometry_msgs::Pose& ref_pose, const ros::Time& stamp) const {
  if (!params_.visualize_topics)
    return;
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
  if (!params_.visualize_topics)
    return;
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

void LocalPlanner::publishFootprints(const ros::Time& stamp) {
  if (!params_.visualize_topics)
    return;
  // Number of waypoints.
  const auto& waypoints = motion_planner_->waypoints();
  const int nwaypoints  = waypoints.size();
  const auto& footprint = footprint::closedPolygon(footprint_);
  // Create the visualization marker.
  visualization_msgs::MarkerArray marker_array_msg;
  marker_array_msg.markers.reserve(std::max(nwaypoints, last_nwaypoints_));
  for (int i = 0; i < nwaypoints; ++i) {
    visualization_msgs::Marker marker;
    marker.header.stamp    = stamp;
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    marker.ns              = ros::this_node::getNamespace() + "LocalPlanner";
    marker.id              = i;
    marker.type            = visualization_msgs::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::Marker::MODIFY;
    marker.pose            = waypoints[i].pose;
    marker.scale.x         = 0.0025;
    marker.color.r         = 0.0;
    marker.color.g         = 0.0;
    marker.color.b         = 0.0;
    marker.color.a         = 0.7;
    marker.points          = footprint;
    marker_array_msg.markers.emplace_back(marker);
  }
  // Delete the old ones.
  for (int i = nwaypoints; i < last_nwaypoints_; ++i) {
    visualization_msgs::Marker delete_marker;
    delete_marker.ns     = ros::this_node::getNamespace() + "LocalPlanner";
    delete_marker.id     = i;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    marker_array_msg.markers.emplace_back(delete_marker);
  }
  last_nwaypoints_ = nwaypoints;
  footprints_pub_.publish(marker_array_msg);
}

void LocalPlanner::publishTwist(
    const geometry_msgs::PoseStamped& actuation_pose,
    const geometry_msgs::Twist& cmd) const {
  if (!params_.visualize_topics)
    return;
  // Actuation pose quantities.
  const geometry_msgs::Point& actuation_point = actuation_pose.pose.position;
  const double yaw = tf::getYaw(actuation_pose.pose.orientation);
  // Twist quantities.
  const double dir    = yaw + std::atan2(cmd.linear.y, cmd.linear.x);
  const double length = std::hypot(cmd.linear.x, cmd.linear.y);
  const double angle  = yaw + M_PI / 2;
  // Marker for linear command.
  visualization_msgs::Marker marker_lin_cmd;
  marker_lin_cmd.id               = 0;
  marker_lin_cmd.header           = actuation_pose.header;
  marker_lin_cmd.ns               = "cmd_navigation";
  marker_lin_cmd.type             = visualization_msgs::Marker::ARROW;
  marker_lin_cmd.action           = visualization_msgs::Marker::MODIFY;
  marker_lin_cmd.pose.position    = actuation_point;
  marker_lin_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(dir);
  marker_lin_cmd.scale.x          = length;
  marker_lin_cmd.scale.y          = 0.035;
  marker_lin_cmd.scale.z          = 0.05;
  marker_lin_cmd.color.r          = 0.0;
  marker_lin_cmd.color.g          = 0.0;
  marker_lin_cmd.color.b          = 1.0;
  marker_lin_cmd.color.a          = 0.5;
  // Marker for angular command.
  visualization_msgs::Marker marker_ang_cmd;
  marker_ang_cmd.id               = 1;
  marker_ang_cmd.header           = actuation_pose.header;
  marker_ang_cmd.ns               = "cmd_navigation";
  marker_ang_cmd.type             = visualization_msgs::Marker::ARROW;
  marker_ang_cmd.action           = visualization_msgs::Marker::MODIFY;
  marker_ang_cmd.pose.position.x  = actuation_point.x + 0.22 * std::cos(yaw);
  marker_ang_cmd.pose.position.y  = actuation_point.y + 0.22 * std::sin(yaw);
  marker_ang_cmd.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  marker_ang_cmd.scale.x          = cmd.angular.z;
  marker_ang_cmd.scale.y          = 0.035;
  marker_ang_cmd.scale.z          = 0.05;
  marker_ang_cmd.color.r          = 0.0;
  marker_ang_cmd.color.g          = 0.0;
  marker_ang_cmd.color.b          = 1.0;
  marker_ang_cmd.color.a          = 0.5;
  // Publish the message.
  visualization_msgs::MarkerArray marker_cmd;
  marker_cmd.markers = {marker_lin_cmd, marker_ang_cmd};
  cmd_pub_.publish(marker_cmd);
}

void LocalPlanner::twistToGlobalFrame(
    const geometry_msgs::Twist& robot_twist,
    geometry_msgs::Twist* map_twist) const {
  const double robot_yaw = tf::getYaw(robot_pose_.pose.orientation);
  // Transform the twist.
  const double c = std::cos(robot_yaw), s = std::sin(robot_yaw);
  map_twist->linear.x  = c * robot_twist.linear.x - s * robot_twist.linear.y;
  map_twist->linear.y  = s * robot_twist.linear.x + s * robot_twist.linear.y;
  map_twist->angular.z = robot_twist.angular.z;
}

void LocalPlanner::twistToRobotFrame(
    const geometry_msgs::Twist& map_twist,
    geometry_msgs::Twist* robot_twist) const {
  const double robot_yaw = tf::getYaw(robot_pose_.pose.orientation);
  // Transform the twist.
  const double c = std::cos(-robot_yaw), s = std::sin(-robot_yaw);
  robot_twist->linear.x  = c * map_twist.linear.x - s * map_twist.linear.y;
  robot_twist->linear.y  = s * map_twist.linear.x + c * map_twist.linear.y;
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
    safe_twist->angular.z =
        std::copysign(params_.max_safe_ang_velocity, twist.angular.z);
}

bool LocalPlanner::isTrajectorySafe(
    const std::vector<geometry_msgs::PoseStamped>& trajectory) const {
  double cum_lin_lookahead = 0.0, cum_ang_lookahead = 0.0;
  for (unsigned int i = 0; i < trajectory.size() - 1; ++i) {
    const auto& curr_waypoint = trajectory[i].pose;
    const auto& next_waypoint = trajectory[i + 1].pose;
    // Compute the cost of the current waypoint.
    const double x             = curr_waypoint.position.x;
    const double y             = curr_waypoint.position.y;
    const double a             = tf::getYaw(curr_waypoint.orientation);
    const double waypoint_cost = costmap_model_->footprintCost(
        x, y, a, footprint_, inscribed_radius_, circumscribed_radius_);
    if (waypoint_cost < 0. || waypoint_cost >= costmap_2d::LETHAL_OBSTACLE)
      return false;
    // Update the lookahead.
    const double dl = math::linearDistance2D(curr_waypoint, next_waypoint);
    const double da = math::angularDistanceYaw(curr_waypoint, next_waypoint);
    if ((cum_lin_lookahead += dl) >= params_.replanning_lin_lookahead ||
        (cum_ang_lookahead += da) >= params_.replanning_ang_lookahead)
      return true;
  }
}

bool LocalPlanner::needReplanning(
    const std::vector<geometry_msgs::PoseStamped>& old_waypoints,
    const std::vector<geometry_msgs::PoseStamped>& new_waypoints) const {
  const bool current_trajectory_suboptimal =
      math::pathLength(new_waypoints) <=
      params_.replanning_path_length_ratio * math::pathLength(new_waypoints);
  const bool need_replanning =
      current_trajectory_suboptimal || !isTrajectorySafe(old_waypoints);
  if (params_.verbose && need_replanning)
    ROS_INFO_STREAM(
        "squirrel_navigation::LocalPlanner: Found a shorter trajectory or "
        "trajectory not safe. Replanning requested.");
  return need_replanning;
}

bool LocalPlanner::newGoal(const geometry_msgs::Pose& pose) const {
  if (!current_goal_)
    return true;
  bool new_position    = math::linearDistance2D(*current_goal_, pose) > 1e-8;
  bool new_orientation = math::angularDistanceYaw(*current_goal_, pose) > 1e-8;
  return new_position || new_orientation;
}

bool LocalPlanner::BaseBrake::spin(geometry_msgs::Twist* cmd) {
  cmd->linear.x = cmd->linear.y = cmd->angular.z = 0.0;
  if (!enable_stamp_)
    return false;
  else if (ros::Time::now() - *enable_stamp_ >= enable_time_) {
    disable();
    return false;
  } else {
    return true;
  }
}

void LocalPlanner::BaseBrake::enable(const ros::Duration& duration) {
  enable_stamp_.reset(new ros::Time(ros::Time::now()));
  enable_time_ = duration;
}

void LocalPlanner::BaseBrake::disable() { enable_stamp_.reset(nullptr); }

LocalPlanner::Params LocalPlanner::Params::defaultParams() {
  using namespace safety;
  // Set default parameters.
  Params params;
  params.odom_topic                 = "/odom";
  params.footprint_topic            = "/squirrel_footprint_observer/footprint";
  params.collision_based_replanning = true;
  params.replanning_lin_lookahead   = 1.0;
  params.replanning_ang_lookahead   = 1.0;
  params.replanning_path_length_ratio = 0.75;
  params.goal_ang_tolerance           = 0.05;
  params.goal_lin_tolerance           = 0.05;
  params.max_safe_lin_velocity        = 0.5;
  params.max_safe_ang_velocity        = 0.7;
  params.max_safe_lin_displacement    = 0.5;
  params.max_safe_ang_displacement    = 1.0;
  params.safety_observers = {ScanObserver::tag, ArmSkinObserver::tag};
  params.visualize_topics = true;
  params.verbose          = false;
  return params;
}

}  // namespace squirrel_navigation
