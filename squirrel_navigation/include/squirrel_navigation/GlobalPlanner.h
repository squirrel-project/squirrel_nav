// GlobalPlanner.h ---
//
// Filename: GlobalPlanner.h
// Description: ARA*/AD* planner with motion primitives
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Sun Jan 17 18:04:06 2016 (+0100)
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
// /*********************************************************************
// *
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2008, Willow Garage, Inc.
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of the Willow Garage nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// * Author: Mike Phillips
// *********************************************************************/

#ifndef SQUIRREL_NAVIGATION_GLOBALPLANNER_H_
#define SQUIRREL_NAVIGATION_GLOBALPLANNER_H_

#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/inflation_layer.h>
#include <nav_core/base_global_planner.h>
#include <navfn/navfn_ros.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <squirrel_navigation_msgs/GlobalPlannerStats.h>
#include <std_msgs/Bool.h>

#include <sbpl/headers.h>

#include <tf/tf.h>

#include <angles/angles.h>

#include "squirrel_navigation/Common.h"
#include "squirrel_navigation/LatticeSCQ.h"
#include "squirrel_navigation/TrajectoryPlanner.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

namespace squirrel_navigation {

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  GlobalPlanner(void);
  GlobalPlanner(std::string, costmap_2d::Costmap2DROS*);
  virtual ~GlobalPlanner(void);

  virtual void initialize(std::string, costmap_2d::Costmap2DROS*);
  virtual bool makePlan(
      const geometry_msgs::PoseStamped&, const geometry_msgs::PoseStamped&,
      std::vector<geometry_msgs::PoseStamped>&);

 private:
  typedef enum { DIJKSTRA, LATTICE } planner_t;

  ros::NodeHandle nh_;

  std::string name_;
  bool initialized_;

  planner_t curr_planner_;
  navfn::NavfnROS* dijkstra_planner_;
  SBPLPlanner* lattice_planner_;
  TrajectoryPlanner* trajectory_;

  EnvironmentNAVXYTHETALAT* env_;

  std::string lattice_planner_type_;

  double allocated_time_;
  double initial_epsilon_;

  std::string environment_type_;
  std::string cost_map_topic_;

  bool forward_search_;
  std::string primitive_filename_;
  int force_scratch_limit_;

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char sbpl_cost_multiplier_;

  costmap_2d::Costmap2DROS* costmap_ros_;

  ros::Publisher traj_xy_pub_, stats_pub_, traj_xyth_pub_;
  ros::Subscriber update_sub_, odom_sub_;

  nav_msgs::Odometry odom_;

  std::vector<geometry_msgs::Point> footprint_;

  // replanning
  std::vector<geometry_msgs::PoseStamped> plan_;
  double heading_lookahead_, lookahead_dijkstra_, lookahead_lattice_;

  bool verbose_;

  std::mutex guard_;

  unsigned char costMapCostToSBPLCost(unsigned char);
  void publishStats(
      int, int, const geometry_msgs::PoseStamped&,
      const geometry_msgs::PoseStamped&);
  void updatePlannerCallback(const std_msgs::Bool::ConstPtr&);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr&);

  inline void publishTrajectory(
      const std::vector<TrajectoryPlanner::Pose2D>* poses) {
    ros::Time traj_stamp = ros::Time::now();

    const size_t n = poses->size();

    nav_msgs::Path gui_plan;
    gui_plan.header.frame_id = "/map";
    gui_plan.header.stamp    = traj_stamp;

    geometry_msgs::PoseArray gui_poses;
    gui_poses.header.frame_id = "/map";
    gui_poses.header.stamp    = traj_stamp;

    gui_poses.poses.resize(n);
    gui_plan.poses.resize(n);
    for (size_t i = 0; i < poses->size(); ++i) {
      geometry_msgs::Pose pose =
          TrajectoryPlanner::Pose2D::toPoseMsg(poses->at(i));
      gui_poses.poses[i]     = pose;
      gui_plan.poses[i].pose = pose;
    }

    traj_xy_pub_.publish(gui_plan);
    traj_xyth_pub_.publish(gui_poses);
  };

  inline bool newGoal(const geometry_msgs::PoseStamped& goal) const {
    return specialEuclideanDistance(trajectory_->getGoal(), goal.pose) > TOLL;
  };
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_GLOBALPLANNER_H_ */
