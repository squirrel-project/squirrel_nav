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
#include "squirrel_navigation/utils/math_utils.h"

#include <ros/node_handle.h>

#include <costmap_2d/cost_values.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>

#include <ompl/tools/config/MagicConstants.h>

#include <sstream>
#include <thread>

PLUGINLIB_DECLARE_CLASS(
    squirrel_navigation, FootprintPlanner,
    squirrel_navigation::FootprintPlanner, nav_core::BaseGlobalPlanner);

namespace squirrel_navigation {

void FootprintPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (init_)
    return;
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<FootprintPlannerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&FootprintPlanner::reconfigureCallback, this, _1, _2));
  // Initialize the state observers.
  const auto costmap = costmap_ros->getCostmap();
  costmap_ros_.reset(costmap_ros);
  costmap_model_.reset(new base_local_planner::CostmapModel(*costmap));
  // Initialize publishers and subscribers.
  plan_pub_      = pnh.advertise<nav_msgs::Path>("plan", 1);
  waypoints_pub_ = pnh.advertise<geometry_msgs::PoseArray>("waypoints", 1);
  footprints_pub_ =
      pnh.advertise<visualization_msgs::MarkerArray>("footprints", 1);
  footprint_sub_ = nh.subscribe(
      params_.footprint_topic, 1, &FootprintPlanner::footprintCallback, this);
  // Initialize the footprint marker.
  initializeFootprintMarker();
  // Initialize the OMPL bounds.
  ompl_state_space_.reset(new ompl::base::SE2StateSpace);
  ompl_bounds_.reset(new ompl::base::RealVectorBounds(2));
  ompl_bounds_->setLow(0, costmap->getOriginX());
  ompl_bounds_->setLow(1, costmap->getOriginY());
  ompl_bounds_->setHigh(0, costmap->getOriginX() + costmap->getSizeInMetersX());
  ompl_bounds_->setHigh(1, costmap->getOriginY() + costmap->getSizeInMetersY());
  initializeOMPLPlanner();
  // Initialization successful.
  init_ = true;
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
  ompl_planner_->as<ompl::geometric::RRTstar>()->setRange(params_.range);
  if (params_.verbose) {
    std::stringstream output;
    ompl_simple_setup_->getPlanner()->printSettings(output);
    ROS_INFO_STREAM("squirrel_navigation::FootprintPlanner: " << output.str());
  }
  ompl_simple_setup_->setup();
  if (ompl_simple_setup_->solve(params_.max_planning_time)) {
    if (params_.verbose)
      ROS_INFO_STREAM(
          "squirrel_navigation::FootprintPlanner: Path successfully found with "
          "RRT*.");
    ompl_simple_setup_->simplifySolution(params_.max_simplification_time);
    auto& solution_path = ompl_simple_setup_->getSolutionPath();
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
    if (params_.waypoints_resolution > 0.) {
      const int min_num_states =
          std::ceil(solution_path.length() / params_.waypoints_resolution);
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
    FootprintPlannerConfig& config, uint32_t level) {
  params_.footprint_topic            = config.footprint_topic;
  params_.collision_check_resolution = config.collision_check_resolution;
  params_.max_planning_time          = config.max_planning_time;
  params_.max_simplification_time    = config.max_simplification_time;
  params_.waypoints_resolution       = config.waypoints_resolution;
  params_.range                      = config.range;
  params_.verbose                    = config.verbose;
}

void FootprintPlanner::footprintCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& footprint) {
  ROS_INFO_STREAM_ONCE(
      "squirrel_navigation::FootprintPlanner: Subscribed to the footprint.");
  // Update the footprint.
  std::unique_lock<std::mutex> lock(footprint_mtx_);
  if (footprint->polygon.points.size() != footprint_.size()) {
    ompl_need_reinitialization_ = true;
  } else {
    ompl_need_reinitialization_ = false;
    for (unsigned int i = 0; i < footprint_.size(); ++i) {
      if (math::linearDistance2D(footprint_[i], footprint->polygon.points[i]) >
          1e-8) {
        ompl_need_reinitialization_ = true;
        break;
      }
    }
    if (!ompl_need_reinitialization_)
      return;
  }
  // Footprint has changed.
  footprint_.clear();
  footprint_.reserve(footprint->polygon.points.size());
  for (const auto& point32 : footprint->polygon.points) {
    geometry_msgs::Point point;
    point.x = point32.x;
    point.y = point32.y;
    footprint_.emplace_back(point);
  }
  footprint_marker_.points = footprint_;
}

void FootprintPlanner::initializeOMPLPlanner() {
  ompl_state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*ompl_bounds_);
  ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(ompl_state_space_));
  ompl_simple_setup_->setStateValidityChecker(boost::bind(
      &FootprintPlanner::checkValidState, this,
      ompl_simple_setup_->getSpaceInformation().get(), _1));
  ompl_simple_setup_->getSpaceInformation()->setStateValidityCheckingResolution(
      params_.collision_check_resolution);
  ompl_planner_.reset(
      new ompl::geometric::RRTstar(ompl_simple_setup_->getSpaceInformation()));
  ompl_simple_setup_->setPlanner(ompl_planner_);
  ompl_need_reinitialization_ = false;
}

bool FootprintPlanner::checkValidState(
    const ompl::base::SpaceInformation* ompl_simple_setup,
    const ompl::base::State* state) {
  if (!ompl_simple_setup->satisfiesBounds(state))
    return false;
  // Get x,y,a from state.
  auto state_se2 = state->as<ompl::base::SE2StateSpace::StateType>();
  const double x = state_se2->getX();
  const double y = state_se2->getY();
  const double a = state_se2->getYaw();
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
    const ros::Time& stamp) {
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
  geometry_msgs::PoseArray pose_array;
  pose_array.poses.reserve(nwaypoints);
  for (const auto& waypoint : waypoints)
    pose_array.poses.emplace_back(waypoint.pose);
  waypoints_pub_.publish(pose_array);
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
    std::vector<geometry_msgs::PoseStamped>* waypoints) {
  waypoints->clear();
  waypoints->reserve(ompl_states.size());
  for (auto ompl_state : ompl_states) {
    auto ompl_state_se2 =
        ompl_state->as<ompl::base::SE2StateSpace::StateType>();
    const double x = ompl_state_se2->getX();
    const double y = ompl_state_se2->getY();
    const double a = ompl_state_se2->getYaw();
    geometry_msgs::PoseStamped waypoint;
    waypoint.pose.position.x  = x;
    waypoint.pose.position.y  = y;
    waypoint.pose.orientation = tf::createQuaternionMsgFromYaw(a);
    waypoints->emplace_back(waypoint);
  }
}

FootprintPlanner::Params FootprintPlanner::Params::defaultParams() {
  Params params;
  params.footprint_topic            = "/footprint_observer/footprint";
  params.collision_check_resolution = 0.05;
  params.waypoints_resolution       = 0.05;
  params.max_planning_time          = 0.5;
  params.max_simplification_time    = 0.5;
  params.range                      = 1.0;
  params.verbose                    = true;
  return params;
}

}  // namespace squirrel_navigation
