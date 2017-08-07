// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
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

#include "squirrel_navigation/footprint_planner.h"
#include "squirrel_navigation/utils/math_utils.h"

#include <ros/node_handle.h>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/tools/config/MagicConstants.h>

#include <cmath>
#include <sstream>
#include <thread>

PLUGINLIB_DECLARE_CLASS(
    squirrel_navigation, FootprintPlanner,
    squirrel_navigation::FootprintPlanner, nav_core::BaseGlobalPlanner);

namespace squirrel_navigation {

FootprintPlanner::FootprintPlanner()
    : params_(Params::defaultParams()),
      init_(false),
      inscribed_radius_(0.),
      circumscribed_radius_(0.),
      last_nwaypoints_(-1) {}

FootprintPlanner::FootprintPlanner(const Params& params)
    : params_(params),
      init_(false),
      inscribed_radius_(0.),
      circumscribed_radius_(0.),
      last_nwaypoints_(-1) {}

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
  ompl_simple_setup_->clear();
  ompl_simple_setup_->setStartAndGoalStates(ss_start, ss_goal);
  // Compute plan.
  ompl_planner_->as<ompl::geometric::RRTConnect>()->setRange(params_.range);
  if (params_.verbose) {
    std::stringstream output;
    ompl_simple_setup_->getPlanner()->printSettings(output);
    ROS_INFO_STREAM("squirrel_navigation::FootprintPlanner: " << output.str());
  }
  ompl_simple_setup_->setup();
  if (ompl_simple_setup_->solve(params_.max_planning_time)) {
    auto& solution_path = ompl_simple_setup_->getSolutionPath();
    if (params_.verbose)
      ROS_INFO_STREAM(
          "squirrel_navigation::FootprintPlanner: RRT-connect successfully "
          "found a path with"
          << solution_path.getStates().size() << ".");
    ompl_simple_setup_->simplifySolution(params_.max_simplification_time);
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

const std::vector<geometry_msgs::Point>& FootprintPlanner::footprint() const {
  return footprint_;
}

void FootprintPlanner::footprintRadii(
    double* inscribed_radius, double* circumscribed_radius) const {
  *inscribed_radius     = inscribed_radius_;
  *circumscribed_radius = circumscribed_radius_;
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
  if (params_.bold_factor != config.bold_factor) {
    const double factor = (1 - config.bold_factor) / (1 - params_.bold_factor);
    rescaleFootprint(factor);
    params_.bold_factor = config.bold_factor;
  }
  // Do not allow backward motion.
  if (params_.allow_backward_motion != config.allow_backward_motion) {
    ompl_need_reinitialization_   = true;
    params_.allow_backward_motion = config.allow_backward_motion;
  }
  // Set verbosity of OMPL.
  if (params_.verbose)
    ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
  else
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
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
  footprint_ = costmap_2d::toPointVector(footprint->polygon);
  costmap_2d::calculateMinAndMaxDistances(
      footprint_, inscribed_radius_, circumscribed_radius_);
  // Update the marker.
  footprint_marker_.points = footprint_;
  // Apply boldness.
  rescaleFootprint(1 - params_.bold_factor);
}

void FootprintPlanner::initializeOMPLPlanner() {
  ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(ompl_state_space_));
  ompl_state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*ompl_bounds_);
  auto& ompl_space_information = ompl_simple_setup_->getSpaceInformation();
  // Set the state validity checker.
  ompl_simple_setup_->setStateValidityChecker(boost::bind(
      &FootprintPlanner::checkValidState, this, ompl_space_information.get(),
      _1));
  ompl_space_information->setStateValidityCheckingResolution(
      params_.collision_check_resolution);
  // Enforce forward motion in case.
  if (!params_.allow_backward_motion)
    ompl_space_information->setMotionValidator(
        boost::make_shared<ForwardDiscreteMotionValidator>(
            ompl_space_information));
  // Set the planner.
  ompl_planner_.reset(new ompl::geometric::PRMstart(ompl_space_information));
  ompl_simple_setup_->setPlanner(ompl_planner_);
  // Initialization flag.
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
  // Check footprint cost.
  const double footprint_cost = costmap_model_->footprintCost(
      x, y, a, footprint_, inscribed_radius_, circumscribed_radius_);
  if (footprint_cost < 0. || footprint_cost >= costmap_2d::LETHAL_OBSTACLE)
    return false;
  return true;
}

void FootprintPlanner::rescaleFootprint(double factor) {
  footprint_ *= factor;
  inscribed_radius_ *= factor;
  circumscribed_radius_ *= factor;
}

void FootprintPlanner::initializeFootprintMarker() {
  footprint_marker_.ns   = ros::this_node::getNamespace() + "/FootprintPlanner";
  footprint_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  footprint_marker_.action  = visualization_msgs::Marker::MODIFY;
  footprint_marker_.scale.x = 0.0025;
  footprint_marker_.color.r = 0.35;
  footprint_marker_.color.g = 0.35;
  footprint_marker_.color.b = 0.35;
  footprint_marker_.color.a = 1.0;
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
  footprints_array.markers.reserve(std::max(nwaypoints, last_nwaypoints_));
  for (int i = 0; i < nwaypoints; ++i) {
    footprint_marker_.header = header;
    footprint_marker_.id     = i;
    footprint_marker_.pose   = waypoints[i].pose;
    footprints_array.markers.emplace_back(footprint_marker_);
  }
  // Clear last waypoints.
  for (int i = nwaypoints; i < last_nwaypoints_; ++i) {
    visualization_msgs::Marker delete_marker;
    delete_marker.ns     = ros::this_node::getNamespace() + "/FootprintPlanner";
    delete_marker.id     = i;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    footprints_array.markers.emplace_back(delete_marker);
  }
  last_nwaypoints_ = nwaypoints;
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
  params.collision_check_resolution = 0.01;
  params.waypoints_resolution       = 0.05;
  params.max_planning_time          = 0.5;
  params.max_simplification_time    = 0.5;
  params.range                      = 1.0;
  params.allow_backward_motion      = false;
  params.verbose                    = true;
  return params;
}

FootprintPlanner::ForwardDiscreteMotionValidator::
    ForwardDiscreteMotionValidator(ompl::base::SpaceInformation* si)
    : ompl::base::MotionValidator(si), discrete_motion_validator_(si) {}

FootprintPlanner::ForwardDiscreteMotionValidator::
    ForwardDiscreteMotionValidator(const ompl::base::SpaceInformationPtr& si)
    : ompl::base::MotionValidator(si), discrete_motion_validator_(si) {}

bool FootprintPlanner::ForwardDiscreteMotionValidator::checkMotion(
    const ompl::base::State* state1, const ompl::base::State* state2) const {
  return isForward(state1, state2) &&
         discrete_motion_validator_.checkMotion(state1, state2);
}

bool FootprintPlanner::ForwardDiscreteMotionValidator::checkMotion(
    const ompl::base::State* state1, const ompl::base::State* state2,
    std::pair<ompl::base::State*, double>& last_valid_state) const {
  return isForward(state1, state2) &&
         discrete_motion_validator_.checkMotion(
             state1, state2, last_valid_state);
}

void FootprintPlanner::ForwardDiscreteMotionValidator::ominus(
    const ompl::base::SE2StateSpace::StateType& state2,
    const ompl::base::SE2StateSpace::StateType& state1, double* x, double* y,
    double* a) const {
  const double dx = state2.getX() - state1.getX();
  const double dy = state2.getY() - state1.getY();
  const double c1 = std::cos(-state1.getYaw());
  const double s1 = std::sin(-state1.getYaw());
  // The output.
  *x = c1 * dx - s1 * dy;
  *y = s1 * dx + c1 * dy;
  *a = state2.getYaw() - state1.getYaw();
}

bool FootprintPlanner::ForwardDiscreteMotionValidator::isForward(
    const ompl::base::State* state1, const ompl::base::State* state2) const {
  // First state.
  auto s1 = state1->as<ompl::base::SE2StateSpace::StateType>();
  auto s2 = state2->as<ompl::base::SE2StateSpace::StateType>();
  // Vectors.
  double ux, uy, ua;
  ominus(*s2, *s1, &ux, &uy, &ua);
  return ux > 0.;
}

std::vector<geometry_msgs::Point>& operator*=(
    std::vector<geometry_msgs::Point>& footprint, double scale) {
  for (auto& point : footprint) {
    point.x *= scale;
    point.y *= scale;
  }
  return footprint;
}

}  // namespace squirrel_navigation
