// PushingPlanner.h --- 
// 
// Filename: PushingPlanner.h
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

// Change Log:
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

#ifndef PUSHINGPLANNER_H_
#define PUSHINGPLANNER_H_

#include <ros/ros.h>
#include <ros/exceptions.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <costmap_2d/cost_values.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "squirrel_rgbd_mapping_msgs/GetPushingPlan.h"

#include <boost/thread.hpp>

namespace squirrel_pushing_planner {

class PushingPlanner
{
 public:
  PushingPlanner( void );
  virtual ~PushingPlanner( void );

  void spin( void );
  
  void waitForPlannerService( void );
  
 private:  
  ros::NodeHandle public_nh_, private_nh_;
  ros::Publisher pushing_plan_pub_;
  ros::ServiceServer get_pushing_plan_srv_;
  ros::Subscriber costmap_sub_;
  
  nav_msgs::Path plan_;
  
  tf::TransformListener tfl_;
  
  std::string node_name_;
  
  // Parameters
  double tolerance_, robot_radius_;
  std::string plan_frame_id_, start_goal_frame_id_;
  int costmap_height_, costmap_width_;

  nav_msgs::OccupancyGrid costmap_;

  cv::Mat* obstacle_map_;
  size_t offset_height_, offset_width_;
  
  boost::mutex costmap_mtx_;
  
  bool getPlan_( squirrel_rgbd_mapping_msgs::GetPushingPlan::Request&, squirrel_rgbd_mapping_msgs::GetPushingPlan::Response& );
  void costmapCallback_( const nav_msgs::OccupancyGrid::ConstPtr& );
  bool isNumericValid_( squirrel_rgbd_mapping_msgs::GetPushingPlan::Request& );
  bool inFootprint_( const geometry_msgs::Polygon&, const geometry_msgs::Point& );
};

}  // namespace squirrel_pushing_planner

#endif /* PUSHINGPLANNER_H_ */

// 
// PushingPlanner.h ends here
