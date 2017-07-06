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

#include "squirrel_navigation/global_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(
    squirrel_navigation, GlobalPlanner, squirrel_navigation::GlobalPlanner,
    nav_core::BaseGlobalPlanner);

namespace squirrel_navigation {

void GlobalPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<GlobalPlannerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&GlobalPlanner::reconfigureCallback, this, _1, _2));
  // Initialize internal observers.
  costmap_ros_.reset(costmap_ros);
  footprint_.reset(nullptr);
  // Initialize the path planners.
  dijkstra_planner_.reset(new navfn::NavfnROS);
  dijkstra_planner_->initialize(name + "/dijkstra", costmap_ros);
  footprint_planner_.reset(new FootPrintPlanner);
  footprint_planner_->initialize(name + "/RRTstar", costmap_ros);
  // Initialize publisher/subscribers.
  footprint_sub_ = nh.subscribe(
      params_.footprint_topic, 1, &GlobalPlanner::footprintCallback, this);
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
    for (int i = 0; i < (int)waypoints.size() - 1; ++i) {
      if (params_.plan_with_constant_heading) {
        waypoints[i].orientation =
            tf::createQuaternionMsgFromYaw(params_.heading);
      } else {
        const double dx = math::delta<0>(waypoints[i - 1], waypoints[i + 1]);
        const double dy = math::delta<1>(waypoints[i - 1], waypoints[i + 1]);
        const double heading     = std::atan2(dy, dx);
        waypoints[i].orientation = tf::createQuaternionMsgFromYaw(heading);
      }
    }
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

}  // namespace squirrel_navigation
