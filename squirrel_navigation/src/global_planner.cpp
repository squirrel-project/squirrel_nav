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

#include "squirrel_navigation/global_planner.h"
#include "squirrel_navigation/utils/math_utils.h"

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(
    squirrel_navigation, GlobalPlanner, squirrel_navigation::GlobalPlanner,
    nav_core::BaseGlobalPlanner);

namespace squirrel_navigation {

void GlobalPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (init_)
    return;
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<GlobalPlannerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&GlobalPlanner::reconfigureCallback, this, _1, _2));
  // Initialize internal observers.
  costmap_ros_.reset(costmap_ros);
  // Initialize the replanning guard.
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  ReplanningGuardInstance::get()->initialize(*costmap);
  // Initialize the path planners.
  dijkstra_planner_.reset(new navfn::NavfnROS);
  dijkstra_planner_->initialize(name + "/Dijkstra", costmap_ros);
  footprint_planner_.reset(new FootprintPlanner);
  footprint_planner_->initialize(name + "/RRTstar", costmap_ros);
  // Initialize the publishers.
  plan_pub_      = pnh.advertise<nav_msgs::Path>("plan", 1);
  waypoints_pub_ = pnh.advertise<geometry_msgs::PoseArray>("poses", 1);
  footprints_pub_ =
      pnh.advertise<visualization_msgs::MarkerArray>("footprints", 1);
  // Initialization successful.
  init_ = true;
  ROS_INFO_STREAM(
      "squirrel_localizer::GlobalPlanner: initialization successful.");
}

bool GlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& waypoints) {
  if (params_.collision_based_replanning) {
    ReplanningGuard* replanning_guard = ReplanningGuardInstance::get();
    double inscribed_radius, circumscribed_radius;
    footprint_planner_->footprintRadii(
        &inscribed_radius, &circumscribed_radius);
    if (params_.plan_with_footprint) {
      replanning_guard->checkCollisionsWithRadius(
          circumscribed_radius, params_.collision_based_replanning_lookahead);
    } else {
      replanning_guard->checkCollisionsWithFootprint(
          footprint_planner_->footprint(), inscribed_radius,
          circumscribed_radius, params_.collision_based_replanning_lookahead);
    }
    if (replanning_guard->replanningFlag())
      return true;
  }
  // Compute a collision free path.
  if (params_.plan_with_footprint) {
    return footprint_planner_->makePlan(start, goal, waypoints);
  } else if (dijkstra_planner_->makePlan(start, goal, waypoints)) {
    waypoints.front() = start;
    for (int i = 1; i < (int)waypoints.size() - 1; ++i) {
      if (params_.plan_with_constant_heading) {
        waypoints[i].pose.orientation =
            tf::createQuaternionMsgFromYaw(params_.heading);
      } else {
        const auto& prev_waypoint = waypoints[i - 1].pose;
        const auto& next_waypoint = waypoints[i + 1].pose;
        const double dx = math::delta<0>(prev_waypoint, next_waypoint);
        const double dy = math::delta<1>(prev_waypoint, next_waypoint);
        waypoints[i].pose.orientation =
            tf::createQuaternionMsgFromYaw(std::atan2(dy, dx));
      }
    }
    waypoints.back() = goal;
    // Print info.
    if (params_.verbose)
      ROS_INFO_STREAM(
          "squirrel_navigation::GlobalPlanner: Found a collision free path ("
          << waypoints.size() << " waypoints).");
    // Publish topics.
    const ros::Time& now = ros::Time::now();
    publishPlan(waypoints, now);
    publishWaypoints(waypoints, now);
    publishFootprints(waypoints, now);
    return true;
  }
  return false;
}

void GlobalPlanner::reconfigureCallback(
    GlobalPlannerConfig& config, uint32_t level) {
  params_.plan_with_footprint        = config.plan_with_footprint;
  params_.plan_with_constant_heading = config.plan_with_constant_heading;
  params_.heading                    = config.heading;
  params_.verbose                    = config.verbose;
  // Collision based replanning.
  params_.collision_based_replanning = config.collision_based_replanning;
  params_.collision_based_replanning_lookahead =
      config.collision_based_replanning_lookahead;
  ReplanningGuardInstance::get()->setEnabled(
      params_.collision_based_replanning_lookahead);
}

void GlobalPlanner::publishPlan(
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    const ros::Time& stamp) const {
  nav_msgs::Path plan_msg;
  plan_msg.header.stamp    = stamp;
  plan_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  plan_msg.poses          = waypoints;
  plan_pub_.publish(plan_msg);
}

void GlobalPlanner::publishWaypoints(
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    const ros::Time& stamp) const {
  geometry_msgs::PoseArray pose_array_msg;
  pose_array_msg.header.stamp    = stamp;
  pose_array_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  pose_array_msg.poses.reserve(waypoints.size());
  for (const auto& waypoint : waypoints)
    pose_array_msg.poses.emplace_back(waypoint.pose);
  waypoints_pub_.publish(pose_array_msg);
}

void GlobalPlanner::publishFootprints(
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    const ros::Time& stamp) const {
  // Number of waypoints and footprint marker.
  const int nwaypoints  = waypoints.size();
  const auto& footprint = closedPolygon(footprint_planner_->footprint());
  // Create the visualization marker.
  visualization_msgs::MarkerArray marker_array_msg;
  marker_array_msg.markers.reserve(nwaypoints);
  for (size_t i = 0; i < nwaypoints; ++i) {
    visualization_msgs::Marker marker;
    marker.header.stamp    = stamp;
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    marker.ns              = ros::this_node::getNamespace();
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
  footprints_pub_.publish(marker_array_msg);
}

std::vector<geometry_msgs::Point> GlobalPlanner::closedPolygon(
    const std::vector<geometry_msgs::Point>& open_polygon) const {
  if (open_polygon.size() <= 1)
    return open_polygon;
  std::vector<geometry_msgs::Point> output = open_polygon;
  output.insert(output.end(), open_polygon.back()); /* maybe not so efficient */
  return output;
}

GlobalPlanner::Params GlobalPlanner::Params::defaultParams() {
  Params params;
  params.collision_based_replanning           = false;
  params.collision_based_replanning_lookahead = 1.0;
  params.plan_with_footprint                  = false;
  params.plan_with_constant_heading           = false;
  params.heading                              = 0.0;
  params.verbose                              = false;
  return params;
}

}  // namespace squirrel_navigation
