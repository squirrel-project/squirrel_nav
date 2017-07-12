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

#include "squirrel_navigation/linear_motion_planner.h"
#include "squirrel_navigation/utils/math_utils.h"

#include <ros/console.h>
#include <ros/node_handle.h>

#include <algorithm>
#include <cassert>
#include <thread>

namespace squirrel_navigation {

void LinearMotionPlanner::initialize(const std::string& name) {
  if (init_)
    return;
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name);
  dsrv_.reset(new dynamic_reconfigure::Server<LinearMotionPlannerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&LinearMotionPlanner::reconfigureCallback, this, _1, _2));
  // Initialization successful.
  init_ = true;
  ROS_INFO_STREAM(
      "squirrel_navigation::LinearMotionPlanner: initialization succesful.");
}

void LinearMotionPlanner::reset(
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    const ros::Time& start) {
  if (waypoints.size() < 2)
    return;
  // Create the velocity profile.
  smoothTrajectory(waypoints, &waypoints_);
  waypoints_[0].header.stamp = start;
  for (unsigned int i = 1; i < waypoints_.size(); ++i) {
    const double dl = math::linearDistance2D(waypoints_[i], waypoints_[i - 1]);
    const double da =
        math::angularDistanceYaw(waypoints_[i], waypoints_[i - 1]);
    const double dt = computeSafetyVelocity(dl, da);
    waypoints_[i].header.stamp =
        waypoints_[i - 1].header.stamp + ros::Duration(dt);
  }
}

void LinearMotionPlanner::update(
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    const ros::Time& stamp) {
  std::unique_lock<std::mutex> lock(update_mtx_);
  std::vector<geometry_msgs::PoseStamped> new_waypoints;
  smoothTrajectory(waypoints, &new_waypoints);
  // Update the trajectory with new waypoints.
  const int head_waypoint_index = computeHeadingWaypointIndex(stamp);
  if (head_waypoint_index + params_.waypoints_heading_lookahead ==
      (int)waypoints_.size() - 1)
    return;
  if ((int)waypoints.size() - 1 < 2 + params_.waypoints_heading_lookahead)
    return;
  // Merge the new trajectory in to the new one.
  const int last_waypoint_index = std::max(0, head_waypoint_index - 1);
  waypoints_.erase(
      waypoints_.begin() + last_waypoint_index +
          params_.waypoints_heading_lookahead,
      waypoints_.end());
  waypoints_.insert(
      waypoints_.end(),
      new_waypoints.begin() + 2 + params_.waypoints_heading_lookahead,
      new_waypoints.end());
  // Recompute the velocity profiles.
  for (unsigned int i =
           head_waypoint_index + params_.waypoints_heading_lookahead + 1;
       i < waypoints_.size(); ++i) {
    const double dl = math::linearDistance2D(waypoints_[i], waypoints_[i - 1]);
    const double da =
        math::angularDistanceYaw(waypoints_[i], waypoints_[i - 1]);
    const double dt = computeSafetyVelocity(dl, da);
    waypoints_[i].header.stamp =
        waypoints_[i - 1].header.stamp + ros::Duration(dt);
  }
}

void LinearMotionPlanner::computeReference(
    const ros::Time& ref_stamp, geometry_msgs::Pose* ref_pose,
    geometry_msgs::Twist* ref_twist) {
  std::unique_lock<std::mutex> lock(update_mtx_);
  const auto next_waypoint             = computeHeadingWaypoint(ref_stamp);
  const auto last_waypoint             = std::prev(next_waypoint);
  const geometry_msgs::Pose& last_pose = last_waypoint->pose;
  const geometry_msgs::Pose& head_pose = next_waypoint->pose;
  const ros::Time& last_stamp          = last_waypoint->header.stamp;
  const ros::Time& head_stamp          = next_waypoint->header.stamp;
  // Constant linear profile for velocity.
  const double delta_stamp = head_stamp.toSec() - last_stamp.toSec();
  ref_twist->linear.x      = math::delta<0>(last_pose, head_pose) / delta_stamp;
  ref_twist->linear.y      = math::delta<1>(last_pose, head_pose) / delta_stamp;
  ref_twist->angular.z     = math::delta<2>(last_pose, head_pose) / delta_stamp;
  // Interpolate waypoints for the reference pose.
  const double alpha = (ref_stamp.toSec() - last_stamp.toSec()) / delta_stamp;
  const double inter = std::min(alpha, 1.);
  ref_pose->position = math::linearInterpolation2D(last_pose, head_pose, inter);
  ref_pose->orientation = math::slerpYaw(last_pose, head_pose, inter);
}

const geometry_msgs::PoseStamped& LinearMotionPlanner::start() const {
  return waypoints_.front();
}

const geometry_msgs::PoseStamped& LinearMotionPlanner::goal() const {
  return waypoints_.back();
}

const std::vector<geometry_msgs::PoseStamped>& LinearMotionPlanner::waypoints()
    const {
  return waypoints_;
}

const geometry_msgs::PoseStamped& LinearMotionPlanner::operator()(int i) const {
  assert(i >= 0 && i < (int)waypoints_.size());
  return waypoints_[i];
}

const geometry_msgs::PoseStamped& LinearMotionPlanner::operator[](int i) const {
  assert(i >= 0 && i < (int)waypoints_.size());
  return waypoints_[i];
}

const geometry_msgs::PoseStamped& LinearMotionPlanner::at(int i) const {
  assert(i >= 0 && i < (int)waypoints_.size());
  return waypoints_[i];
}

void LinearMotionPlanner::reconfigureCallback(
    LinearMotionPlannerConfig& config, uint32_t level) {
  params_.max_linear_velocity         = config.max_linear_velocity;
  params_.max_angular_velocity        = config.max_angular_velocity;
  params_.linear_smoother             = config.linear_smoother;
  params_.angular_smoother            = config.angular_smoother;
  params_.time_scaler                 = config.time_scaler;
  params_.waypoints_heading_lookahead = config.waypoints_heading_lookahead;
}

double LinearMotionPlanner::computeSafetyVelocity(
    double linear_delta, double angular_delta) const {
  const double lin_time = linear_delta / params_.max_linear_velocity;
  const double ang_time = angular_delta / params_.max_angular_velocity;
  return params_.time_scaler * std::max(lin_time, ang_time);
}

std::vector<geometry_msgs::PoseStamped>::const_iterator
    LinearMotionPlanner::computeHeadingWaypoint(const ros::Time& stamp) const {
  geometry_msgs::PoseStamped query_pose;
  query_pose.header.stamp   = stamp;
  const auto heading_waypoint_ptr = std::upper_bound(
      waypoints_.begin(), waypoints_.end(), query_pose,
      [](const geometry_msgs::PoseStamped& lhs,
         const geometry_msgs::PoseStamped& rhs) {
        return lhs.header.stamp.toSec() < rhs.header.stamp.toSec();
      });
  return heading_waypoint_ptr != waypoints_.end()
             ? heading_waypoint_ptr
             : std::prev(heading_waypoint_ptr);
}

int LinearMotionPlanner::computeHeadingWaypointIndex(
    const ros::Time& stamp) const {
  return std::distance(waypoints_.begin(), computeHeadingWaypoint(stamp));
}

void LinearMotionPlanner::smoothTrajectory(
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    std::vector<geometry_msgs::PoseStamped>* smooth_waypoints) const {
  smooth_waypoints->clear();
  if (waypoints.empty())
    return;
  const int nwaypoints = waypoints.size();
  smooth_waypoints->reserve(nwaypoints);
  smooth_waypoints->emplace_back(waypoints.front());
  for (int i = 1; i < nwaypoints - 1; ++i) {
    geometry_msgs::PoseStamped waypoint;
    waypoint.pose.position = math::linearInterpolation2D(
        smooth_waypoints->at(i - 1), waypoints[i], params_.linear_smoother);
    waypoint.pose.orientation = math::slerpYaw(
        smooth_waypoints->at(i - 1), waypoints[i], params_.angular_smoother);
    smooth_waypoints->emplace_back(waypoint);
  }
  smooth_waypoints->emplace_back(waypoints.back());
}

LinearMotionPlanner::Params LinearMotionPlanner::Params::defaultParams() {
  Params params;
  params.max_linear_velocity         = 0.5;
  params.max_angular_velocity        = 0.7;
  params.linear_smoother             = 0.95;
  params.angular_smoother            = 0.95;
  params.time_scaler                 = 0.75;
  params.waypoints_heading_lookahead = 10;
  return params;
}

}  // namespace squirrel_navigation
