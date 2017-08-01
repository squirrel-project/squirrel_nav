// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
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

#include "squirrel_navigation/global_planner.h"
#include "squirrel_navigation/utils/math_utils.h"

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
  // Initialize the path planners.
  dijkstra_planner_.reset(new navfn::NavfnROS);
  dijkstra_planner_->initialize(name + "/Dijkstra", costmap_ros);
  footprint_planner_.reset(new FootprintPlanner);
  footprint_planner_->initialize(name + "/RRTstar", costmap_ros);
  // Initialization successful.
  init_ = true;
  ROS_INFO_STREAM(
      "squirrel_localizer::GlobalPlanner: initialization successful.");
}

bool GlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& waypoints) {
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
}

GlobalPlanner::Params GlobalPlanner::Params::defaultParams() {
  Params params;
  params.plan_with_footprint        = false;
  params.plan_with_constant_heading = false;
  params.heading                    = 0.0;
  params.verbose                    = false;
  return params;
}

}  // namespace squirrel_navigation
