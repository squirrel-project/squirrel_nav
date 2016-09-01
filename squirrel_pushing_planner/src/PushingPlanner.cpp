// PushingPlanner.cpp --- 
// 
// Filename: PushingPlanner.cpp
// Description: Planner for the pushing task
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Mon Dec  8 13:36:41 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Tue Nov 24 14:37:47 2015 (+0100)
//           By: Federico Boniardi
//     Update #: 1
// URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Copyright (c) 2014, Federico Boniardi
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
// 
// 

// Code:

#include "squirrel_pushing_planner/PushingPlanner.h"

namespace squirrel_pushing_planner {

PushingPlanner::PushingPlanner( void )  :
    OBSTACLE(100),
    tolerance_(1.0),
    robot_radius_(0.22),
    start_goal_frame_id_("/map"),
    plan_frame_id_("/odom"),
    object_frame_id_("/base_link"),
    obstacles_map_(NULL),
    costmap_topic_("/move_base/global_costmap/costmap"),
    costmap_updates_topic_("/move_base/global_costmap/costmap_updates"),
    obstacles_map_ready_(false),
    node_name_(ros::this_node::getName())
{
  ros::NodeHandle pnh("~");
  
  pnh.param<double>("tolerance", tolerance_, 1.0);
  pnh.param<std::string>("plan_frame_id", plan_frame_id_, "/odom");
  pnh.param<std::string>("start_goal_frame_id", start_goal_frame_id_, "/map");
  pnh.param<std::string>("object_frame_id", object_frame_id_,"/base_link");
  pnh.param<std::string>("costmap_topic", costmap_topic_, "/move_base/global_costmap/costmap");
  pnh.param<std::string>("costmap_updates_topic", costmap_updates_topic_, "/move_base/global_costmap/costmap_updates");

  plan_.header.frame_id = plan_frame_id_;
  
  pushing_plan_pub_ = pnh.advertise<nav_msgs::Path>("pushingPlan", 1000);

  get_pushing_plan_srv_ = nh_.advertiseService("getPushingPlan", &PushingPlanner::getPlan, this); 
  costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &PushingPlanner::costmapCallback, this);
  costmap_updates_sub_ = nh_.subscribe("/move_base/global_costmap/costmap_updates", 1, &PushingPlanner::costmapUpdatesCallback, this);

  nh_.param<double>("/move_base/global_costmap/inflation_layer/inflation_radius", inflation_radius_, 0.6);
  nh_.param<double>("/move_base/global_costmap/robot_radius", robot_radius_, 0.16);
}

PushingPlanner::~PushingPlanner( void )
{
  if ( obstacles_map_ )
    delete obstacles_map_;
  
  pushing_plan_pub_.shutdown();
  get_pushing_plan_srv_.shutdown();
  costmap_sub_.shutdown();
  costmap_updates_sub_.shutdown();
  nh_.shutdown();
}

void PushingPlanner::spin( void )
{
  ros::Rate lr(2.0);
  while ( ros::ok() ) {     
    try {
      ros::spinOnce();
      lr.sleep();
    } catch ( std::runtime_error& err ) {
      ROS_ERROR("%s: %s", node_name_.c_str(), err.what());
      std::exit(1);
    }
  }
}

void PushingPlanner::waitForPlannerService( void )
{
 //  if ( !ros::service::waitForService("/move_base/make_plan", ros::Duration(60.0)) ) {
//     ROS_ERROR("%s: Service [/move_base/make_plan] unavailable, shutting down the node...", node_name_.c_str());
//     ros::shutdown();
//   }
}

void PushingPlanner::updateCostmap( double offset ) const
{
  bool inscribed_radius_updated, inflation_radius_updated;
  
  { // Updating the inscribed radius
    dynamic_reconfigure::Config config;
    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse res;
    dynamic_reconfigure::DoubleParameter param;

    param.name = "inscribed_radius";
    param.value = robot_radius_ + offset;
    config.doubles.push_back(param);

    req.config = config;

    inscribed_radius_updated = ros::service::call("/move_base/global_costmap/inflation_layer/set_parameters", req, res);
  }

  { // Updating the inflation radius
    dynamic_reconfigure::Config config;

    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse res;
    dynamic_reconfigure::DoubleParameter param;

    param.name = "inflation_radius";
    param.value = inflation_radius_ + offset;
    config.doubles.push_back(param);

    req.config = config;

    inflation_radius_updated = ros::service::call("/move_base/global_costmap/inflation_layer/set_parameters", req, res);
  }

  if ( not inscribed_radius_updated )
    ROS_WARN_STREAM( node_name_ << ": Unable to update parameters /move_base/global_costmap/inflation_layer/inscribed_radius.");
  if ( not inflation_radius_updated )
    ROS_WARN_STREAM( node_name_ << ": Unable to update parameters /move_base/global_costmap/inflation_layer/inflation_radius.");
}

bool PushingPlanner::getPlan( squirrel_rgbd_mapping_msgs::GetPushingPlan::Request& req,
                              squirrel_rgbd_mapping_msgs::GetPushingPlan::Response& res )
{
  if ( !isNumericValid(req) ) {
    std::string node_name = ros::this_node::getName();
    ROS_ERROR("%s: got an invalid request. Cannot create a planner for pushing", node_name.c_str() );
    return false;
  }

  // Get start
  geometry_msgs::PoseStamped start, start_m;
  start.header.frame_id = start_goal_frame_id_;
  start.pose.orientation = tf::createQuaternionMsgFromYaw(req.start.theta);
  start.pose.position.x = req.start.x;
  start.pose.position.y = req.start.y;
  
  try {
    tfl_.waitForTransform(start_goal_frame_id_, "/map", ros::Time::now(), ros::Duration(1.0));
    tfl_.transformPose("/map", start, start_m);
    start_m.header.frame_id = "/map";
  } catch ( tf::TransformException& ex ) {
    ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
    updateCostmap(0.0);
    return false;
  }

  double start_theta_m = tf::getYaw(start_m.pose.orientation);
  
  // GetPlan goal
  geometry_msgs::PoseStamped goal, goal_m;
  goal.header.frame_id = start_goal_frame_id_;
  goal.pose.orientation = tf::createQuaternionMsgFromYaw(req.goal.theta);
  goal.pose.position.x = req.goal.x;
  goal.pose.position.y = req.goal.y;
  
  try {
    tfl_.waitForTransform(start_goal_frame_id_, "/map", ros::Time::now(), ros::Duration(1.0));
    tfl_.transformPose("/map", goal, goal_m);
  } catch ( tf::TransformException& ex ) {
    ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
    updateCostmap(0.0);
    return false;
  }

  double object_radius = getObjectRadius(req.object);
  updateCostmap(object_radius);
  updateCostmap(object_radius);

  // Create call to move_base service
  nav_msgs::GetPlan plan;
  plan.request.tolerance = tolerance_;
  plan.request.start = start_m;
  plan.request.goal = goal_m;
  
  if ( ros::service::call("/move_base/make_plan", plan) ) {
    nav_msgs::Path res_plan = plan.response.plan;
    res.plan.poses.resize(res_plan.poses.size());

    // computing the clearance
    if ( not obstacles_map_ ) {
      ROS_WARN("%s: Costmap is not ready yet. Unable to compute path's clearance.", ros::this_node::getName().c_str());
      res.clearance = -1.0;
    } else {
      boost::unique_lock<boost::mutex> lock(obstacles_mtx_);
      
      cv::Mat dist_map;
      double min, max;
      
      cv::distanceTransform(*obstacles_map_, dist_map, CV_DIST_L2, 3);
      cv::minMaxLoc(dist_map, &min, &max);
      cv::normalize(dist_map, dist_map, 0.0, max*costmap_info_.resolution, cv::NORM_MINMAX);

      res.clearance = std::numeric_limits<float>::max();
      for (size_t k=0; k<res_plan.poses.size(); ++k) {
        unsigned int i = res_plan.poses[k].pose.position.x/costmap_info_.resolution;
        unsigned int j = res_plan.poses[k].pose.position.y/costmap_info_.resolution;
        
        res.clearance = std::min(dist_map.at<float>(costmap_info_.height/2-j-1,costmap_info_.height/2+i),(float)res.clearance); 
      }
    }

    try {
      ros::Time now = ros::Time::now();
      tfl_.waitForTransform(start_goal_frame_id_, "/map", ros::Time::now(), ros::Duration(1.0));
      for (size_t i=0; i<res_plan.poses.size(); ++i) {
        res_plan.poses[i].header.frame_id = "/map"; // just to be sure, move_base/getPlan returns in /map
        tfl_.transformPose(start_goal_frame_id_, res_plan.poses[i], res.plan.poses[i]);
        res.plan.poses[i].header.frame_id = start_goal_frame_id_;
        res.plan.poses[i].header.stamp = now;
      }

      res.plan.header.frame_id = start_goal_frame_id_;
      res.plan.header.stamp = now;
      
    } catch ( tf::TransformException& ex ) {
      ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
      updateCostmap(0.0);
      return false;
    }

    updateCostmap(0.0);
    
    return true;
  } else {
    ROS_WARN("%s: Cannot compute a valid plan.", ros::this_node::getName().c_str());
    return false;
  }
}

void PushingPlanner::costmapCallback( const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg )
{
  if ( not obstacles_map_ ) {
    costmap_info_.map_load_time = costmap_msg->info.map_load_time;
    costmap_info_.resolution = costmap_msg->info.resolution;
    costmap_info_.width = costmap_msg->info.width;
    costmap_info_.height = costmap_msg->info.height;
    costmap_info_.origin.position.x = costmap_msg->info.origin.position.x;
    costmap_info_.origin.position.y = costmap_msg->info.origin.position.y;
    costmap_info_.origin.position.z = costmap_msg->info.origin.position.z;
    costmap_info_.origin.orientation.x = costmap_msg->info.origin.orientation.x;
    costmap_info_.origin.orientation.y = costmap_msg->info.origin.orientation.y;
    costmap_info_.origin.orientation.z = costmap_msg->info.origin.orientation.z;
    costmap_info_.origin.orientation.w = costmap_msg->info.origin.orientation.w;

    obstacles_map_ = new cv::Mat(costmap_info_.height,costmap_info_.width,CV_LOAD_IMAGE_GRAYSCALE);

    for (size_t i=0; i<costmap_info_.height; ++i) {
      for (size_t j=0; j<costmap_info_.width; ++j) {
        if ( costmap_msg->data[costmap_info_.width*i+j] == OBSTACLE ) {
          obstacles_map_->at<uchar>(costmap_info_.height-i-1,j) = 0;
        } else
          obstacles_map_->at<uchar>(costmap_info_.height-i-1,j) = 255;
      }
    }

    obstacles_map_ready_ = true;
  }
  
  ROS_INFO("%s: Got obstacles map.", ros::this_node::getName().c_str());
}

void PushingPlanner::costmapUpdatesCallback( const map_msgs::OccupancyGridUpdate::ConstPtr& updates_msg )
{
  if ( (not obstacles_map_) or (not obstacles_map_ready_) )
    return;

  int center_i = updates_msg->x;
  int center_j = updates_msg->y;
  unsigned int width = updates_msg->width;
  unsigned int height = updates_msg->height;
  
  
  for (unsigned int i=-height/2; i<height/2; ++i) {
    for (unsigned int j=-width/2; j<width/2; ++j) {
      unsigned int x = costmap_info_.height-(i+center_i)-1;
      unsigned int y = j+center_j;
      obstacles_map_->at<uchar>(x,y) = 255*(updates_msg->data[width*i+j]!=OBSTACLE);
    }
  }
  
  return;
}
    
bool PushingPlanner::isNumericValid( squirrel_rgbd_mapping_msgs::GetPushingPlan::Request& req ) const
{
  bool is_valid_start = !std::isnan(req.start.x) and !std::isinf(req.start.x)
      and !std::isnan(req.start.y) and !std::isinf(req.start.y)
      and !std::isnan(req.start.theta) and !std::isinf(req.start.theta);

  bool is_valid_goal = !std::isnan(req.goal.x) and !std::isinf(req.goal.x)
      and !std::isnan(req.goal.y) and !std::isinf(req.goal.y)
      and !std::isnan(req.start.theta) and !std::isinf(req.goal.theta);

  return (is_valid_start and is_valid_goal);
}

bool PushingPlanner::inFootprint( const geometry_msgs::Polygon& polygon, const geometry_msgs::Point& p ) const
{
  const unsigned int n = polygon.points.size()-1;

  int cn = 0;
  for (int i=0; i<n; ++i) {    
    if ( ((polygon.points[i].y <= p.y) and (polygon.points[i+1].y > p.y)) or
         ((polygon.points[i].y > p.y) and (polygon.points[i+1].y <=  p.y)) )
    { 
      double vt = (double)(p.y  - polygon.points[i].y) / (polygon.points[i+1].y - polygon.points[i].y);
      if ( p.x < polygon.points[i].x+vt*(polygon.points[i+1].x-polygon.points[i].x) ) 
        ++cn; 
    }
  }
  
  return !(cn%2);
}

double PushingPlanner::getObjectRadius( const geometry_msgs::Polygon& poly ) const
{
  const size_t nvertices = poly.points.size();

  geometry_msgs::Point32 mean;

  for (size_t i=0; i<nvertices; ++i) {
    mean.x += poly.points[i].x/nvertices;
    mean.y += poly.points[i].y/nvertices;
  }

  double radius = -std::numeric_limits<double>::max();
  for (size_t i=0; i<nvertices; ++i) {
    double dist = linearDistance(mean,poly.points[i]);
    radius = dist > radius ? dist : radius;
  }

  return radius;
}
   
}  // namespace squirrel_pushing_planner

// 
// PushingPlanner.cpp ends here
