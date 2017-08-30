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

#include "squirrel_navigation/footprint_planner.h"
#include "squirrel_navigation/utils/math_utils.h"

#include <ros/init.h>
#include <ros/node_handle.h>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>

#include <boost/filesystem.hpp>

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
      inflation_layer_(nullptr),
      footprint_changed_(true),
      last_nwaypoints_(-1),
      inscribed_radius_(0.),
      circumscribed_radius_(0.) {}

FootprintPlanner::FootprintPlanner(const Params& params)
    : params_(params),
      init_(false),
      inflation_layer_(nullptr),
      footprint_changed_(true),
      last_nwaypoints_(-1),
      inscribed_radius_(0.),
      circumscribed_radius_(0.) {}

void FootprintPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (init_)
    return;
  // Initialize the parameter server.
  ros::NodeHandle pnh("~/" + name), nh;
  dsrv_.reset(new dynamic_reconfigure::Server<FootprintPlannerConfig>(pnh));
  dsrv_->setCallback(
      boost::bind(&FootprintPlanner::reconfigureCallback, this, _1, _2));
  pnh.param<std::string>(
      "motion_primitives_filename", motion_primitives_url_,
      "motion_primitives.mprim");
  if (!boost::filesystem::exists(motion_primitives_url_))
    ROS_ERROR_STREAM(
        "squirrel_navigation/FootprintPlanner: The file '"
        << motion_primitives_url_
        << "' does not exist. The footprint planner might not work.");
  // Initialize the state observers.
  costmap_ros_     = costmap_ros;
  inflation_layer_ = getInflationLayer(costmap_ros_);
  // Initialize publishers and subscribers.
  plan_pub_      = pnh.advertise<nav_msgs::Path>("plan", 1);
  waypoints_pub_ = pnh.advertise<geometry_msgs::PoseArray>("waypoints", 1);
  footprints_pub_ =
      pnh.advertise<visualization_msgs::MarkerArray>("footprints", 1);
  footprint_sub_ = nh.subscribe(
      params_.footprint_topic, 1, &FootprintPlanner::footprintCallback, this);
  // Initialize the footprint marker.
  initializeFootprintMarker();
  // Initialization successful.
  sbpl_need_reinitialization_ = true;
  init_                       = true;
  ROS_INFO_STREAM(
      "squirrel_navigation/FootprintPlanner: initialization successful.");
}

bool FootprintPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& waypoints) {
  std::unique_lock<std::mutex> lock(footprint_mtx_);
  if (sbpl_need_reinitialization_)
    initializeSBPLPlanner();
  // Update the costmap.
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  updateSBPLCostmap(*costmap);
  // Set starting pose.
  try {
    const double start_x = start.pose.position.x - costmap->getOriginX();
    const double start_y = start.pose.position.y - costmap->getOriginY();
    const double start_a = tf::getYaw(start.pose.orientation);
    // Set the starting state.
    const int ret = sbpl_env_->SetStart(start_x, start_y, start_a);
    if (ret <= 0 || sbpl_planner_->set_start(ret) == 0) {
      ROS_ERROR_STREAM(
          "squirrel_navigation/FootprintPlanner: Unable to set the start.");
      return false;
    }
  } catch (sbpl::Exception* ex) {
    ROS_ERROR_STREAM(
        "squirrel_navigation/FootprintPlanner: Something went wrong while "
        "setting the start. "
        << ex->what());
    return false;
  }
  // Set goal pose.
  try {
    const double goal_x = goal.pose.position.x - costmap->getOriginX();
    const double goal_y = goal.pose.position.y - costmap->getOriginY();
    const double goal_a = tf::getYaw(goal.pose.orientation);
    // Set the starting state.
    const int ret = sbpl_env_->SetGoal(goal_x, goal_y, goal_a);
    if (ret <= 0 || sbpl_planner_->set_goal(ret) == 0) {
      ROS_ERROR_STREAM(
          "squirrel_navigation::FootprintPlanner: Unable to set the goal.");
      return false;
    }
  } catch (sbpl::Exception* ex) {
    ROS_ERROR_STREAM(
        "squirrel_navigation::FootprintPlanner: Something went wrong while "
        "setting the goal. "
        << ex->what());
    return false;
  }
  // Compute the plan.
  int solution_cost, ret;
  std::vector<int> solution_states_ids;
  std::vector<sbpl::Pose> sbpl_waypoints;
  try {
    sbpl_planner_->set_initialsolution_eps(params_.initial_epsilon);
    ret = sbpl_planner_->replan(
        params_.max_planning_time, &solution_states_ids, &solution_cost);
    sbpl_env_->ConvertStateIDPathintoXYThetaPath(
        &solution_states_ids, &sbpl_waypoints);
  } catch (sbpl::Exception* ex) {
    ROS_ERROR_STREAM(
        "squirrel_navigation/FootprintPlanner: Something went wrong while "
        "planning. "
        << ex->what());
    return false;
  }
  if (!ret) {
    if (params_.verbose)
      ROS_WARN_STREAM(
          "squirrel_navigation/FootprintPlanner: Unable to find a collision "
          "free path.");
    return false;
  } else {
    convertSBPLStatesToWayPoints(sbpl_waypoints, &waypoints);
    publishPath(waypoints, ros::Time::now());
    if (params_.verbose)
      ROS_WARN_STREAM(
          "squirrel_navigation/FootprintPlanner: Found a collision free path "
          "with "
          << waypoints.size() << " waypoints.");
    return true;
  }
  return false;
}

void FootprintPlanner::reconfigureCallback(
    FootprintPlannerConfig& config, uint32_t level) {
  params_.footprint_topic   = config.footprint_topic;
  params_.max_planning_time = config.max_planning_time;
  params_.initial_epsilon   = config.initial_epsilon;
  params_.visualize_topics  = config.visualize_topics;
  params_.verbose           = config.verbose;
  if (params_.forward_search != config.forward_search) {
    params_.forward_search = config.forward_search;
    footprint_changed_     = true;
  }
}

const std::vector<geometry_msgs::Point>& FootprintPlanner::footprint() const {
  return footprint_;
}

void FootprintPlanner::footprintCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& footprint) {
  ROS_INFO_STREAM_ONCE(
      "squirrel_navigation/FootprintPlanner: Subscribed to the footprint.");
  // Update the footprint.
  std::unique_lock<std::mutex> lock(footprint_mtx_);
  if (footprint->polygon.points.size() != footprint_.size()) {
    footprint_changed_          = true;
    sbpl_need_reinitialization_ = true;
  } else {
    footprint_changed_ = false;
    for (unsigned int i = 0; i < footprint_.size(); ++i) {
      if (math::linearDistance2D(footprint_[i], footprint->polygon.points[i]) >
          0.01) {
        footprint_changed_ = true;
        break;
      }
    }
    if (!footprint_changed_)
      return;
  }
  // Footprint has changed.
  sbpl_need_reinitialization_ = true;
  footprint_                  = costmap_2d::toPointVector(footprint->polygon);
  costmap_2d::calculateMinAndMaxDistances(
      footprint_, inscribed_radius_, circumscribed_radius_);
  // Update the marker.
  footprint_marker_.points = footprint_;
}

void FootprintPlanner::initializeSBPLPlanner() {
  sbpl_env_.reset(new sbpl::NavigationEnvironment);
  // The global costmap;
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  // Set inscribed radius cost parameter.
  if (!sbpl_env_->SetEnvParameter(
          "cost_inscribed_thresh", costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
    ROS_ERROR_STREAM(
        "squirrel_navigation/FootprintPlanner: Unable to set "
        "'cost_inscribed_thresh' parameter'");
    ros::shutdown();
  }
  // Set circumscibed radius cost parameter.
  if (inflation_layer_ &&
      !sbpl_env_->SetEnvParameter(
          "cost_possibly_circumscribed_thresh",
          inflation_layer_->computeCost(
              circumscribed_radius_ / costmap->getResolution()))) {
    ROS_ERROR_STREAM(
        "squirrel_navigation/FootprintPlanner: Unable to set "
        "'cost_possibly_circumscribed_thresh' parameter'. Is "
        "costmap_2d::InflationLayer initialized?");
    ros::shutdown();
  }
  // Initialize the environment.
  bool initialization_status = false;
  try {
    initialization_status = sbpl_env_->InitializeEnv(
        costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), nullptr, 0., 0.,
        0., 0., 0., 0., 0., 0., 0., sbpl::footprint(footprint_),
        costmap_ros_->getCostmap()->getResolution(), 1.0, 1.0,
        costmap_2d::LETHAL_OBSTACLE, motion_primitives_url_.c_str());
  } catch (sbpl::Exception* ex) {
    ROS_ERROR_STREAM(
        "squirrel_navigation/FootprintPlanner: Something went wrong during "
        "initialization of spbl::NavigationEnvironment. "
        << ex->what() << ". Does the resolution in the motion primitives file "
                         "match the resolution of the map?");
    ros::shutdown();
  }
  // If initialization fails, throw everything away.
  if (!initialization_status) {
    ROS_ERROR_STREAM(
        "squirrel_navigation/FootprintPlanner: Initialization failed.");
    ros::shutdown();
  }
  // Initialize the planner.
  sbpl_planner_.reset(
      new sbpl::ARAstar(sbpl_env_.get(), params_.forward_search));
  sbpl_planner_->set_search_mode(false);
  // Initialization guard.
  sbpl_need_reinitialization_ = false;
}

void FootprintPlanner::updateSBPLCostmap(const costmap_2d::Costmap2D& costmap) {
  for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i)
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j)
      sbpl_env_->UpdateCost(i, j, costmap.getCost(i, j));
}

boost::shared_ptr<costmap_2d::InflationLayer>
    FootprintPlanner::getInflationLayer(costmap_2d::Costmap2DROS* costmap_ros) {
  const auto costmap_plugins = costmap_ros->getLayeredCostmap()->getPlugins();
  for (const auto& layer : *costmap_plugins)
    if (auto inflation_layer =
            boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(layer))
      return inflation_layer;
  return nullptr;
}

void FootprintPlanner::initializeFootprintMarker() {
  footprint_marker_.ns   = ros::this_node::getNamespace() + "FootprintPlanner";
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
  if (!params_.visualize_topics)
    return;
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
    delete_marker.ns     = ros::this_node::getNamespace() + "FootprintPlanner";
    delete_marker.id     = i;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    footprints_array.markers.emplace_back(delete_marker);
  }
  last_nwaypoints_ = nwaypoints;
  footprints_pub_.publish(footprints_array);
}

void FootprintPlanner::convertSBPLStatesToWayPoints(
    const std::vector<sbpl::Pose>& sbpl_states,
    std::vector<geometry_msgs::PoseStamped>* waypoints) {
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  // Fill the waypoints.
  waypoints->clear();
  waypoints->reserve(sbpl_states.size());
  for (const auto& sbpl_state : sbpl_states) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id  = costmap_ros_->getGlobalFrameID();
    pose.pose.position.x  = sbpl_state.x + costmap->getOriginX();
    pose.pose.position.y  = sbpl_state.y + costmap->getOriginY();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(sbpl_state.theta);
    waypoints->emplace_back(pose);
  }
}

FootprintPlanner::Params FootprintPlanner::Params::defaultParams() {
  Params params;
  params.footprint_topic   = "/squirrel_footprint_observer/footprint";
  params.forward_search    = true;
  params.max_planning_time = 0.2;
  params.initial_epsilon   = 0.05;
  params.visualize_topics  = true;
  params.verbose           = false;
  return params;
}

}  // namespace squirrel_navigation
