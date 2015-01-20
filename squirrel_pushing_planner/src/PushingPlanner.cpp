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

#include <ros/exceptions.h>

#include <tf/tf.h>

#include <geometry_msgs/Quaternion.h>

#include <climits>
#include <cmath>
#include <stdexcept>

namespace squirrel_pushing_planner {

PushingPlanner::PushingPlanner( void )  :
    private_nh_("~"),
    tolerance_(1.0),
    robot_radius_(0.22)
{
  node_name_ = ros::this_node::getName();
  
  private_nh_.param("tolerance", tolerance_, 1.0);
  private_nh_.param("robot_radius", robot_radius_, 0.22);
  pushing_plan_pub_ = private_nh_.advertise<nav_msgs::Path>("pushingPlan", 1000);
  get_pushing_plan_srv_ = public_nh_.advertiseService("getPushingPlan", &PushingPlanner::getPlan, this); 
}

PushingPlanner::~PushingPlanner( void )
{
  pushing_plan_pub_.shutdown();
  get_pushing_plan_srv_.shutdown();
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
  if ( !ros::service::waitForService("/move_base/make_plan", ros::Duration(10.0)) ) {
    ROS_ERROR("%s: Service [/move_base/make_service] unavailable, shutting down the node...", node_name_.c_str());
    ros::shutdown();
  }
}

bool PushingPlanner::getPlan( squirrel_rgbd_mapping_msgs::GetPushingPlan::Request& req,
                              squirrel_rgbd_mapping_msgs::GetPushingPlan::Response& res )
{
  nav_msgs::GetPlan plan;

  geometry_msgs::Quaternion q_start = tf::createQuaternionMsgFromYaw(req.start.theta);
  geometry_msgs::Quaternion q_goal = tf::createQuaternionMsgFromYaw(req.goal.theta);

  double object_d = std::numeric_limits<double>::min();
  for (unsigned int i=0; i<req.object.points.size(); ++i) {
    if ( req.object.points[i].x > object_d ) {
      object_d = req.object.points[i].x;
    }
  }
  
  // GetPlan tolerance
  plan.request.tolerance = tolerance_;

  // GetPlan start
  plan.request.start.header.frame_id = "/map";
  plan.request.start.pose.orientation.x = q_start.x;
  plan.request.start.pose.orientation.y = q_start.y;
  plan.request.start.pose.orientation.z = q_start.z;
  plan.request.start.pose.orientation.w = q_start.w;
  plan.request.start.pose.position.x = req.start.x + object_d * std::cos(req.start.theta);
  plan.request.start.pose.position.y = req.start.y + object_d * std::sin(req.start.theta);
  
  // GetPlan goal
  plan.request.goal.header.frame_id = "/map";
  plan.request.goal.pose.orientation.x = q_goal.x;
  plan.request.goal.pose.orientation.y = q_goal.y;
  plan.request.goal.pose.orientation.z = q_goal.z;
  plan.request.goal.pose.orientation.w = q_goal.w;
  plan.request.goal.pose.position.x = req.goal.x;
  plan.request.goal.pose.position.y = req.goal.y;  

  // ROS_INFO("start: (%f, %f, %f)", req.start.x, req.start.y, req.start.theta);
  
  if ( ros::service::call("/move_base/make_plan", plan) ) {
    res.plan.header.frame_id = "/map";
    if ( plan.response.plan.poses.empty() ) {
      ROS_WARN("got empty plan");
      return false;
    } else {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "/map";

      // Add the starting point
      p.pose.position.x = req.start.x;
      p.pose.position.y = req.start.y;
      p.pose.orientation.x = q_start.x;
      p.pose.orientation.y = q_start.y;
      p.pose.orientation.z = q_start.z;
      p.pose.orientation.w = q_start.w;
      res.plan.poses.push_back(p);

      // smooth connectio to the path
      int smoother = 1;
      for (double t=0.5*object_d; t<object_d; ++smoother, t+=object_d*std::pow(0.5, smoother)) {
        double dx_s = plan.response.plan.poses[smoother].pose.position.x - plan.response.plan.poses[smoother-1].pose.position.x;
        double dy_s = plan.response.plan.poses[smoother].pose.position.y - plan.response.plan.poses[smoother-1].pose.position.y;
        p.pose.position.x = res.plan.poses[smoother-1].pose.position.x + object_d * std::pow(0.5, smoother) * std::cos(req.start.theta) + dx_s;
        p.pose.position.y = res.plan.poses[smoother-1].pose.position.y + object_d * std::pow(0.5, smoother) * std::sin(req.start.theta) + dy_s;
        p.pose.orientation.x = q_start.x;
        p.pose.orientation.y = q_start.y;
        p.pose.orientation.z = q_start.z;
        p.pose.orientation.w = q_start.w;
        res.plan.poses.push_back(p);
      }

      // ROS_INFO_STREAM("path size: " << plan.response.plan.poses.size() << " smoother: " << smoother);
      res.plan.poses.insert(res.plan.poses.end(), plan.response.plan.poses.begin()+smoother+1, plan.response.plan.poses.end());
      pushing_plan_pub_.publish(res.plan);
      return true;
    }
  } else {
    ROS_ERROR("failed to call service /move_base/make_plan");
  }
}

}  // namespace squirrel_pushing_planner

// 
// PushingPlanner.cpp ends here
