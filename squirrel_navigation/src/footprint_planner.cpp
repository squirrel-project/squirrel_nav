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

#include "squirrel_navigation/footprint_planner.h"

#include <pluginlib/class_list_macros.h>

#include <costmap_2d/cost_values.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

#include <thread>

PLUGIN_EXPORT_CLASS(
    squirreL_navigation, FootprintPlanner,
    squirrel_navigation::FootprintPlanner, nav_core::BaseGlobalPlanner);

namespace squirrel_navigation {

void FootprintPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  const auto costmap = costmap_ros->getCostmap();
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<FootPrintPlannerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&FootprintPlanner::reconfigureCallback, this, _1, _2));
  // Initialize the state observers.
  costmap_ros_.reset(costmap_ros);
  costmap_model_.reset(new base_local_planner::CostmapModel(*costmap));
  // Publishers and subscribers.
  plan_pub_      = pnh.advertise<nav_msgs::Path>("plan", 1);
  waypoints_pub_ = pnh.advertise<geometry_msgs::PoseArray>("waypoints", 1);
  footprint_pub_ =
      pnh.advertise<visualization_msgs::MarkerArray>("footprints", 1);
  footprint_sub_ = nh.subscribe(
      params_.footprint_topic, 1, &FootprintPlanner::footprintCallback, this);
  // Initialize the footprint marker.
  initializeFootprintMarker();
  // Initialize the OMPL bounds.
  ompl_state_space_.reset(new ompl::base::SE2StateSpace);
  bounds_.reset(new ompl::base::RealVectorBounds(2));
  bounds_->setLow(0, costmap->getOriginX());
  bounds_->setLow(1, costmap->getOriginY());
  bounds_->setHigh(0, costmap->getOriginX() + costmap->getSizeInMetersX());
  bounds_->setHigh(1, costmap->getOriginY() + costmap->getSizeInMetersY());
  initializeOMPLPlanner();
  ROS_INFO_STREAM(
      "squirrel_localizer::FootprintPlanner: initialization successful.");
}

bool FootprintPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& waypoints) {
  std::unique_lock<std::mutex> lock(footprint_mtx_);
  // Reset the Ompl State.
  if (ompl_need_reinitialization_)
    initializeOMPLPlanner();
  ompl::base::ScopedState<> ss_start(ompl_state_space_);
  ompl::base::ScopedState<> ss_goal(ompl_state_space_);
  ss_start = std::vector<double>{start.pose.position.x, start.pose.position.y,
                                 tf::getYaw(start.pose.orientation)};
  ss_goal = std::vector<double>{goal.pose.position.x, goal.pose.position.y,
                                tf::getYaw(goal.pose.orientation)};
  ompl_simple_setup_->setStartAndGoalStates(ss_start, ss_goal);
  // Compute plan.
  ompl_rrt_planner_->setRange(params_.range);
  if (params_.verbose)
    ompl_simple_setup_->getPlanner()->printSettings(std::cout);
  ompl_simple_setup_->setup();
  if (ompl_simple_setup_->solve(params_.max_planning_time)) {
    if (params_.verbose)
      ROS_INFO_STREAM(
          "squirrel_navigation::FootprintPlanner: Path successfully found with "
          "RRT*.");
    ompl_simple_setup_->simplifySolution(params_.max_simplification_time);
    const auto& solution_path = ompl_simpl_setup_->getSolutionPath();
    // Check for validity and repair if possible.
    const std::pair<bool, bool>& path_validity =
        solution_path.checkAndRepair(ompl::magic::MAX_VALID_SAMPLE_ATTEMPTS);
    if (!path_validity.second) {
      ROS_WARN_STREAM(
          "squirrel_navigation::FootprintPlanner: Path may slightly touch an "
          "invalid region of space. Replanning.");
      return false;
    } else if (!path_validity.first) {
      ROS_INFO_STREAM(
          "squirrel_navigation::FootprintPlanner: Path was slightly touching "
          "an invalid region. Now Fixed.");
    }
    // Upsample path.
    if (params_.map_resolution > 0.) {
      const int min_num_states =
          std::ceil(solution_path.length() / params_.map_resolution_);
      solution_path.interpolate(min_num_states);
    }
    // Convert to ROS path.
    convertOMPLStatesToWayPoints(solution_path.getStates(), &waypoints);
    publishPath(waypoints, ros::Time::now());
    return !waypoints.empty();
  } else {
    if (params_.verbose)
      ROS_INFO_STREAM(
          "squirrel_navigation::FoorPrintPlanner: Unable to find a valid "
          "path.");
    return false;
  }
}

void FootprintPlanner::reconfigureCallback(
    FootprintPlanner& config, uint32_t level) {
  params_.footprint_topic            = config.footprint_topic;
  params_.collision_check_resolution = config.collision_check_resolution;
  params_.max_planning_time          = config.max_planning_time;
  params_.max_simplification_time    = config.max_simplification_time;
  params_.map_resolution             = config.map_resolution;
  params_.range                      = config.range;
  params_.verbose                    = config.verbose;
}

void FootprintPlanner::footprintCallback(
    const geometry_msgs::Polygon::ConstPtr& footprint) {
  ROS_INFO_STREAM_ONCE(
      "squirrel_navigation::FootprintPlanner: Subscribed to the footprint.");
  // Update the footprint.
  std::unique_lock<std::mutex> lock(footprint_mtx_);
  if (footprint->points.size() != footprint_.size()) {
    ompl_need_reinitialziation_ = true;
  } else {
    ompl_need_reinitialization_ = false;
    for (unsigned int i = 0; i < footprint_.size(); ++i) {
      const double dx = footprint_[i].x - footprint->points[i].x;
      const double dy = footprint_[i].y - footprint->points[i].y;
      if (std::hypot(dx, dy) > 1e-8) {
        ompl_need_reinitialization_ = true;
        break;
      }
    }
    if (!ompl_need_reinitialization_)
      return;
  }
  // Footprint has changed.
  footprint_.clear();
  footprint_.reserve(footprint->points.size());
  for (const auto& point32 : point) {
    geometry_msgs::Point point;
    point.x = point32.x;
    point.y = point32.y;
    footprint_.emplace_back(point);
  }
  footprint_marker_.points = footprint_;
}

void FootprintPlanner::initializeOMPLPlanner() {
  ompl_state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*bounds_);
  ompl_simple_setup_.reset(new ompl::base::SimpleSetup(ompl_state_space_));
  ompl_simple_setup_->setStateValidityChecker(boost::bind(
      &FootprintPlanner::checkValidState,
      ompl_simple_setup_->getSpaceInformation().get(), _1));
  ompl_simple_setup_->getSpaceInformation()->setStateValidityCheckingResolution(
      params_.collision_check_resolution);
  ompl_rrt_planner_.reset(
      new ompl::base::RRTstar(ompl_simple_setup_->getSpaceInformation()));
  ompl_simple_setup_->setPlanner(ompl_rrt_planner_);
  ompl_need_reinitialization_ = false;
}

void FootprintPlanner::checkValidState(
    const ompl::base::SpaceInformation* ompl_simple_setup,
    const ompl::base::State* state) {
  if (!ompl_simple_setup->satisfiesBounds(state))
    return false;
  // Get x,y,a from state.
  const auto state_se2 = state->as<ompl::base::SE2StateSpace::StateType>();
  double x, y, a;
  state_se2->getX(x);
  state_se2->getY(y);
  state_se2->getYaw(a);
  // Compute wheter footprint is colliding or not.
  return costmap_model_->footprintCost(x, y, a, footprint_) >= 0.;
}

void FootprintPlanner::initializeFootprintMarker() {
  footprint_marker_.type    = visualization_msgs::Marker::LINE_STRIP;
  footprint_marker_.action  = visualization_msgs::Marker::ADD;
  footprint_marker_.color.r = 0.75;
  footprint_marker_.color.g = 0.75;
  footprint_marker_.color.b = 0.75;
  footprint_marker_.color.a = 1.0;
  footprint_marker_.scale.x = 0.1;
}

void FootprintPlanner::publishPath(
    const std::vector<geometry_msgs::PoseStamped>& waypoints,
    const ros::Time& stamp) const {
  const int nwaypoints = waypoints.size();
  // Initialize header.
  std_msgs::Header header;
  header.stamp    = stamp;
  header.frame_id = costmap_ros_->getGlobalFrameID();
  // Publish plan.
  nav_msgs::Path plan;
  plan.header = header;
  plan.poses  = waypoints;
  plan_pub_.publish(plan);
  // Publish waypoints.
  geometry_msgs::PoseArray poses;
  poses.reserve(nwaypoints);
  for (const auto& waypoint : waypoints)
    poses.poses.emplace_back(waypoint.pose);
  waypoints_pub_.publish(poses);
  // Publish the footprints.
  visualization_msgs::MarkerArray footprints_array;
  footprints_array.markers.reserve(nwaypoints);
  for (int i = 0; i < nwaypoints; ++i) {
    footprint_marker_.header = header;
    footprint_marker_.id     = i;
    footprint_marker_.pose   = waypoints[i].pose;
    footprints_array.markers.emplace_back(footprint_marker_);
  }
  footprints_pub_.publish(footprints_array);
}

void FootprintPlanner::convertOMPLStatesToWayPoints(
    const std::vector<ompl::base::State*>& ompl_states,
    std::vector<geometry_msgs::PoseStamped>* waypoints) const {
  waypoints->clear();
  waypoints->reserve(waypoints.size());
  for (auto ompl_state : ompl_states) {
    auto ompl_state_se2 =
        ompl_state->as<ompl::Base::SE2StateSpace::StateType>();
    double x, y, a;
    ompl_state_se2->getX(x);
    ompl_state_se2->getY(y);
    ompl_state_se2->getYaw(a);
    geometry_msgs::PoseStamped waypoint;
    waypoint.pose.position.x  = x;
    waypoint.pose.position.y  = y;
    waypoint.pose.orientation = tf::createQuaternionMsgFromYaw(a);
    waypoints->emplace_back(waypoint);
  }
}

}  // namespace squirrel_navigation
