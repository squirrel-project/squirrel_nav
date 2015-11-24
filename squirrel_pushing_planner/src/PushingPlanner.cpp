// PushingPlanner.cpp --- 
// 
// Filename: PushingPlanner.cpp
// Description: Planner for the pushing task
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Mon Dec  8 13:36:41 2014 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
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
    private_nh_("~"),
    tolerance_(1.0),
    robot_radius_(0.22),
    start_goal_frame_id_("/map"),
    plan_frame_id_("/odom"),
    object_frame_id_("/base_link"),
    obstacles_map_(NULL),
    costmap_topic_("/move_base/global_costmap/costmap"),
    costmap_updates_topic_("/move_base/global_costmap/costmap_updates"),
    obstacles_map_ready_(false)
{
  node_name_ = ros::this_node::getName();
  
  private_nh_.param<double>("tolerance", tolerance_, 1.0);
  private_nh_.param<double>("robot_radius", robot_radius_, 0.22);
  private_nh_.param<std::string>("plan_frame_id", plan_frame_id_, "/odom");
  private_nh_.param<std::string>("start_goal_frame_id", start_goal_frame_id_, "/map");
  private_nh_.param<std::string>("object_frame_id", object_frame_id_,"/base_link");
  private_nh_.param<std::string>("costmap_topic", costmap_topic_, "/move_base/global_costmap/costmap");
  private_nh_.param<std::string>("costmap_updates_topic", costmap_updates_topic_, "/move_base/global_costmap/costmap_updates");

  plan_.header.frame_id = plan_frame_id_;
  
  pushing_plan_pub_ = private_nh_.advertise<nav_msgs::Path>("pushingPlan", 1000);
  get_pushing_plan_srv_ = public_nh_.advertiseService("getPushingPlan", &PushingPlanner::getPlan_, this); 
  costmap_sub_ = public_nh_.subscribe("/move_base/global_costmap/costmap", 1, &PushingPlanner::costmapCallback_, this);
  costmap_updates_sub_ = public_nh_.subscribe("/move_base/global_costmap/costmap_updates", 1, &PushingPlanner::costmapUpdatesCallback_, this);
}

PushingPlanner::~PushingPlanner( void )
{
  if ( obstacles_map_ )
    delete obstacles_map_;
  
  pushing_plan_pub_.shutdown();
  get_pushing_plan_srv_.shutdown();
  costmap_sub_.shutdown();
  costmap_updates_sub_.shutdown();
  private_nh_.shutdown();
  public_nh_.shutdown();
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
  if ( !ros::service::waitForService("/move_base/make_plan", ros::Duration(60.0)) ) {
    ROS_ERROR("%s: Service [/move_base/make_service] unavailable, shutting down the node...", node_name_.c_str());
    ros::shutdown();
  }
}

bool PushingPlanner::getPlan_( squirrel_rgbd_mapping_msgs::GetPushingPlan::Request& req,
                               squirrel_rgbd_mapping_msgs::GetPushingPlan::Response& res )
{
  if ( !isNumericValid_(req) ) {
    std::string node_name = ros::this_node::getName();
    ROS_ERROR("%s: got an invalid request. Cannot create a planner for pushing", node_name.c_str() );
    return false;
  }

  nav_msgs::GetPlan plan;

  // Transform starting position in map frame
  geometry_msgs::Quaternion q_start = tf::createQuaternionMsgFromYaw(req.start.theta);

  geometry_msgs::PoseStamped start, start_m;
  start.header.frame_id = start_goal_frame_id_;
  start.pose.orientation = q_start;
  start.pose.position.x = req.start.x;
  start.pose.position.y = req.start.y;

  try {
    tfl_.waitForTransform(start_goal_frame_id_, "/map", ros::Time::now(), ros::Duration(1.0));
    tfl_.transformPose("/map", start, start_m);
    start_m.header.frame_id = "/map";
  } catch ( tf::TransformException& ex ) {
    ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
    return true;
  }

  double start_theta_m = tf::getYaw(start_m.pose.orientation);
  
  // Transform polygon in map frame
  const size_t n = req.object.points.size();
  
  geometry_msgs::Polygon object_m;
  for (unsigned int i=0; i<n; ++i) {
    geometry_msgs::PoseStamped vertex, vertex_stamped_m;
    vertex.header.frame_id = object_frame_id_;
    vertex.pose.position.x = req.object.points[i].x;
    vertex.pose.position.y = req.object.points[i].y;
    vertex.pose.orientation.w = 1.0;
    
    try {
      tfl_.waitForTransform(object_frame_id_, "/map", ros::Time::now(), ros::Duration(1.0));
      tfl_.transformPose("/map", vertex, vertex_stamped_m);
      vertex_stamped_m.header.frame_id = "/map";
    } catch ( tf::TransformException& ex ) {
      ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
      return true;
    }

    geometry_msgs::Point32 vertex_m;
    vertex_m.x = vertex_stamped_m.pose.position.x;
    vertex_m.y = vertex_stamped_m.pose.position.y;
    
    object_m.points.push_back(vertex_m);
  }

  // Getting info from polygon000
  double object_d = -std::numeric_limits<double>::max();
  double object_min_x = std::numeric_limits<double>::max(),
      object_max_x = -std::numeric_limits<double>::max(),
      object_min_y = std::numeric_limits<double>::max(),
      object_max_y = -std::numeric_limits<double>::max();

  for (unsigned int i=0; i<n; ++i) {
    // Computing the farthest distance from the starting point
    geometry_msgs::Point32 p,q;
    p.x = object_m.points[i].x - start_m.pose.position.x;
    p.y = object_m.points[i].y - start_m.pose.position.y;
    q.x = std::cos(start_theta_m);
    q.y = std::sin(start_theta_m);
    
    double d = dot_(p,q);
    if ( d > object_d )
      object_d = d;

    // Computing the patch on the object to remove obstacles
    if ( object_m.points[i].x < object_min_x ) 
      object_min_x = object_m.points[i].x;
    if ( object_m.points[i].x > object_max_x ) 
      object_max_x = object_m.points[i].x;
    if ( object_m.points[i].y < object_min_y ) 
      object_min_y = object_m.points[i].y;
    if ( object_m.points[i].y > object_max_y ) 
      object_max_y = object_m.points[i].y;
  }

  unsigned int patch_i_start = object_min_x/costmap_info_.resolution;
  unsigned int patch_i_end = object_max_x/costmap_info_.resolution;
  unsigned int patch_j_start = object_min_y/costmap_info_.resolution;
  unsigned int patch_j_end = object_max_y/costmap_info_.resolution;
    
  if ( (req.object.points[0].x != req.object.points[n-1].x) or
       (req.object.points[0].y != req.object.points[n-1].y)  )
    req.object.points.push_back(req.object.points[0]);

    // GetPlan tolerance
  plan.request.tolerance = tolerance_;

  // GetPlan start
  geometry_msgs::PoseStamped start_real;
  start_real.header.frame_id = start_goal_frame_id_;
  start_real.pose.orientation = q_start;
  start_real.pose.position.x = req.start.x + object_d*std::cos(req.start.theta);
  start_real.pose.position.y = req.start.y + object_d*std::sin(req.start.theta);

  try {
    tfl_.waitForTransform(start_goal_frame_id_, "/map", ros::Time::now(), ros::Duration(1.0));
    tfl_.transformPose("/map", start_real, plan.request.start);
    plan.request.start.header.frame_id = "/map";
  } catch ( tf::TransformException& ex ) {
    ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
    return true;
  }
  
  // GetPlan goal
  geometry_msgs::Quaternion q_goal = tf::createQuaternionMsgFromYaw(req.goal.theta);

  geometry_msgs::PoseStamped goal;

  goal.header.frame_id = start_goal_frame_id_;
  goal.pose.orientation = q_goal;
  goal.pose.position.x = req.goal.x;
  goal.pose.position.y = req.goal.y;  

  try {
    tfl_.waitForTransform(start_goal_frame_id_, "/map", ros::Time::now(), ros::Duration(1.0));
    tfl_.transformPose("/map", goal, plan.request.goal);
    plan.request.goal.header.frame_id = "/map";
  } catch ( tf::TransformException& ex ) {
    ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
    return false;
  }
  
  if ( ros::service::call("/move_base/make_plan", plan) ) {
    if ( plan.response.plan.poses.empty() ) {
      ROS_WARN("%s: got empty plan", node_name_.c_str());
      return true;
    } else {
      if( !plan_.poses.empty() ) {
        plan_.poses.clear();
      }
      
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "/map";

      // Add the starting point
      p.pose.position.x = start_m.pose.position.x;
      p.pose.position.y = start_m.pose.position.y;
      p.pose.orientation = start_m.pose.orientation;
      plan_.poses.push_back(p);

      // Adjust move base approximation
      plan.response.plan.poses[0].pose.position.x = plan.request.start.pose.position.x;
      plan.response.plan.poses[0].pose.position.y = plan.request.start.pose.position.y;

      // smooth connection to the path
      double c = 0.5;
      double toll = 1e-4;
      int smoother = 1;
      for (double t=0.5*object_d; (t<object_d-toll && smoother<plan.response.plan.poses.size()); ++smoother, t+=object_d*std::pow(c,smoother)) {
        double dx_s = plan.response.plan.poses[smoother].pose.position.x - plan.response.plan.poses[smoother-1].pose.position.x;
        double dy_s = plan.response.plan.poses[smoother].pose.position.y - plan.response.plan.poses[smoother-1].pose.position.y;
        p.pose.position.x = plan_.poses[smoother-1].pose.position.x + object_d*std::pow(c,smoother) * std::cos(start_theta_m) + dx_s;
        p.pose.position.y = plan_.poses[smoother-1].pose.position.y + object_d*std::pow(c,smoother) * std::sin(start_theta_m) + dy_s;
        p.pose.orientation = start_m.pose.orientation;
        plan_.poses.push_back(p);
      }
      
      plan_.poses.insert(plan_.poses.end(), plan.response.plan.poses.begin()+smoother+1, plan.response.plan.poses.end());

      if ( not obstacles_map_ ) {
        ROS_WARN("%s: Costmap is not ready yet. Unable to compute path's clearance.", ros::this_node::getName().c_str());
        res.clearance = -1.0;
      } else {
        boost::unique_lock<boost::mutex> lock(obstacles_mtx_);
        
        for (unsigned int i=patch_i_start; i<patch_i_end; ++i) {
          for (unsigned int j=patch_j_start; j<patch_j_end; ++j) {
            geometry_msgs::Point p;
            p.x = i*costmap_info_.resolution;
            p.y = j*costmap_info_.resolution;
            if ( inFootprint_(object_m,p) ) {
              obstacles_map_->at<uchar>(costmap_info_.height/2-j-1,costmap_info_.height/2+i) = 255;
            }
          }
        }
        
        cv::Mat dist_map;
        double min, max;
        
        cv::distanceTransform(*obstacles_map_, dist_map, CV_DIST_L2, 3);
        cv::minMaxLoc(dist_map, &min, &max);
        cv::normalize(dist_map, dist_map, 0.0, max*costmap_info_.resolution, cv::NORM_MINMAX);
        
        res.clearance = std::numeric_limits<float>::max();
        for (size_t k=0; k<plan_.poses.size(); ++k) {
          unsigned int i = plan_.poses[k].pose.position.x/costmap_info_.resolution;
          unsigned int j = plan_.poses[k].pose.position.y/costmap_info_.resolution;

          res.clearance = std::min(dist_map.at<float>(costmap_info_.height/2-j-1,costmap_info_.height/2+i),(float)res.clearance); 
        }
      }      
      
      // Create response in plan_frame_id_
      res.plan.header.frame_id = plan_frame_id_;
      for (unsigned int i=0; i<plan_.poses.size(); ++i) { 
        try {
          geometry_msgs::PoseStamped p;
          tfl_.waitForTransform("/map", plan_frame_id_, ros::Time::now(), ros::Duration(0.5));
          tfl_.transformPose(plan_frame_id_, plan_.poses[i], p);
          p.header.frame_id = plan_frame_id_;
          res.plan.poses.push_back(p);
        } catch (tf::TransformException& ex) {
          ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
          return false;
        }
      }
      
      pushing_plan_pub_.publish(res.plan);
      return true;
    }
  } else {
    ROS_ERROR("%s: call to service [/move_base/make_plan] failed.", node_name_.c_str());
    return false;
  }
}

void PushingPlanner::costmapCallback_( const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg )
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

void PushingPlanner::costmapUpdatesCallback_( const map_msgs::OccupancyGridUpdate::ConstPtr& updates_msg )
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
    
bool PushingPlanner::isNumericValid_( squirrel_rgbd_mapping_msgs::GetPushingPlan::Request& req )
{
  bool is_valid_start = !std::isnan(req.start.x) and !std::isinf(req.start.x)
      and !std::isnan(req.start.y) and !std::isinf(req.start.y)
      and !std::isnan(req.start.theta) and !std::isinf(req.start.theta);

  bool is_valid_goal = !std::isnan(req.goal.x) and !std::isinf(req.goal.x)
      and !std::isnan(req.goal.y) and !std::isinf(req.goal.y)
      and !std::isnan(req.start.theta) and !std::isinf(req.goal.theta);

  return (is_valid_start and is_valid_goal);
}

bool PushingPlanner::inFootprint_( const geometry_msgs::Polygon& polygon, const geometry_msgs::Point& p )
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

}  // namespace squirrel_pushing_planner

// 
// PushingPlanner.cpp ends here
