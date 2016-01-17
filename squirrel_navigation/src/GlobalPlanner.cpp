// GlobalPlanner.cpp --- 
// 
// Filename: GlobalPlanner.cpp
// Description: Global planner using ARA*/AD* on motion primitive lattice
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Sun Jan 17 18:22:25 2016 (+0100)
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

// Code:

#include "squirrel_navigation/GlobalPlanner.h"

PLUGINLIB_DECLARE_CLASS(squirrel_navigation, GlobalPlanner, squirrel_navigation::GlobalPlanner, nav_core::BaseGlobalPlanner);

namespace squirrel_navigation {

GlobalPlanner::GlobalPlanner( void ) :
    initialized_(false),
    costmap_ros_(NULL),
    dijkstra_planner_(NULL),
    lattice_planner_(NULL),
    curr_planner_(DIJKSTRA)
{
  ROS_INFO("squirrel_localizer::GlobalPlanner started.");
}

GlobalPlanner::GlobalPlanner( std::string name, costmap_2d::Costmap2DROS* costmap_ros ) :
    initialized_(false),
    costmap_ros_(NULL),
    dijkstra_planner_(NULL),
    lattice_planner_(NULL),
    curr_planner_(DIJKSTRA),
    name_(name)
{
  initialize(name, costmap_ros);
}

GlobalPlanner::~GlobalPlanner( void )
{
  if ( dijkstra_planner_ )
    delete dijkstra_planner_;
  if ( lattice_planner_ )
    delete lattice_planner_;
  if ( env_ )
    delete env_;
}

void GlobalPlanner::initialize( std::string name, costmap_2d::Costmap2DROS* costmap_ros )
{
  if( not initialized_ ) {
    ros::NodeHandle pnh("~|"+name);
    pnh.param<bool>("verbose", verbose_, false);

    // Initializing the Dijkstra planner
    dijkstra_planner_ = new navfn::NavfnROS(name+"/dijkstra",costmap_ros);
    
    // Initializing the Lattice Planner
    ros::NodeHandle pnh_l("~/"+name+"/lattice");

    pnh_l.param<std::string>("planner_type", lattice_planner_type_, "ARAPlanner");
    pnh_l.param<std::string>("environment_type", environment_type_, "XYThetaLattice");
    pnh_l.param<std::string>("primitive_filename", primitive_filename_, "");
    pnh_l.param<double>("allocated_time", allocated_time_, 10.0);
    pnh_l.param<double>("initial_epsilon",initial_epsilon_,3.0);
    pnh_l.param<int>("force_scratch_limit", force_scratch_limit_, 500);
    pnh_l.param<bool>("forward_search", forward_search_, false);
    
    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    pnh_l.param<double>("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    pnh_l.param<double>("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

    int lethal_obstacle;
    pnh_l.param<int>("lethal_obstacle",lethal_obstacle,20);
    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_-1;
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
    if ( verbose_ )
      ROS_INFO("%s: [SBPL] lethal: %uz, inscribed inflated: %uz, multiplier: %uz", name.c_str(), lethal_obstacle, inscribed_inflated_obstacle_, sbpl_cost_multiplier_);

    costmap_ros_ = costmap_ros;

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    if ( "XYThetaLattice" == environment_type_ ) {
      if ( verbose_ )
        ROS_INFO_STREAM(name << ": Using a 3D costmap for theta lattice");
      env_ = new EnvironmentNAVXYTHETALAT();
    } else {
      ROS_WARN_STREAM( name << ": Reset to XYThetaLattice as is currently the only supported environment." );
      env_ = new EnvironmentNAVXYTHETALAT();
    }

    // check if the costmap has an inflation layer
    unsigned char cost_possibly_circumscribed_tresh = 0;
    for(std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
        layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end();
        ++layer) {
      boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);

      if ( not inflation_layer )
        continue;

      cost_possibly_circumscribed_tresh = inflation_layer->computeCost(costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
    }

    if( not env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) ) {
      ROS_ERROR_STREAM(name << ": Failed to set cost_inscribed_thresh parameter" );
      std::exit(EXIT_FAILURE);
    }

    if( not env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost_(cost_possibly_circumscribed_tresh)) ){
      ROS_ERROR_STREAM( name << ": Failed to set cost_possibly_circumscribed_thresh parameter" );
      std::exit(EXIT_FAILURE);
    }
    
    int obst_cost_thresh = costMapCostToSBPLCost_(costmap_2d::LETHAL_OBSTACLE);
    std::vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint.size());
    for (size_t ii(0); ii < footprint.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint[ii].x;
      pt.y = footprint[ii].y;
      perimeterptsV.push_back(pt);
    }

    bool ret;
    try{
      ret = env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
                                costmap_ros_->getCostmap()->getSizeInCellsY(), // height
                                0, // mapdata
                                0, 0, 0, // start (x, y, theta, t)
                                0, 0, 0, // goal (x, y, theta)
                                0, 0, 0, //goal tolerance
                                perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs,
                                timetoturn45degsinplace_secs, obst_cost_thresh,
                                primitive_filename_.c_str());
    } catch( SBPL_Exception e ){
      ROS_ERROR_STREAM( name << ": [SBPL] Encountered a fatal exception!" );
      ret = false;
    }
    
    if( not ret ) {
      ROS_ERROR_STREAM( name << ": [SBPL] initialization failed!");
      std::exit(1);
    }
    
    for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
        env_->UpdateCost(ix, iy, costMapCostToSBPLCost_(costmap_ros_->getCostmap()->getCost(ix,iy)));

    if ( "ARAPlanner"  == lattice_planner_type_ ) {
      ROS_INFO_STREAM( name << ": Planning with ARA*" );
      lattice_planner_ = new ARAPlanner(env_, forward_search_);
    } else if ( "ADPlanner" == lattice_planner_type_ ) {
      ROS_INFO_STREAM( name << ": Planning with AD*" );
      lattice_planner_ = new ADPlanner(env_, forward_search_);
    } else{
      ROS_ERROR_STREAM(name << ": ARAPlanner and ADPlanner are currently the only supported planners!" );
      std::exit(EXIT_FAILURE);
    }

    ROS_INFO_STREAM(name << ": Initialized successfully" );
    plan_pub_ = pnh.advertise<nav_msgs::Path>("plan", 1);
    stats_pub_ = pnh.advertise<squirrel_nav_msgs::GlobalPlannerStats>("lattice_planner_stats", 1);

    update_sub_ = nh_.subscribe("/plan_with_footprint", 1, &GlobalPlanner::updatePlannerCallback_, this);
    
    initialized_ = true;
  }
}

bool GlobalPlanner::makePlan( const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan )
{
  if ( not initialized_ ) {
    ROS_ERROR_STREAM(name_ << ": Global planner is not initialized");
    return false;
  }

  plan.clear();

  if ( verbose_ ) {
    ROS_INFO("%s: Getting start point (%g,%g) goal point (%g,%g)", name_.c_str(), start.pose.position.x,
             start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  }

  boost::unique_lock<boost::mutex> lock(all_);
  
  nav_msgs::Path gui_path;
  
  switch ( curr_planner_ ) {
    case DIJKSTRA: {
      dijkstra_planner_->makePlan(start, goal, plan);

      // Publish the path
      ros::Time plan_time = ros::Time::now();
      
      gui_path.poses.resize(plan.size());
      gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
      gui_path.header.stamp = plan_time;
      gui_path.poses = plan;
      
      break;
    }  

    case LATTICE: {
      double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
      double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

      try{
        int ret = env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
        if ( ret < 0 or lattice_planner_->set_start(ret) == 0 ) {
          ROS_ERROR_STREAM( name_ << ": Failed to set start state" );
          return false;
        }
      } catch( SBPL_Exception e ) {
        ROS_ERROR_STREAM(name_ << ": [SBPL] encountered a fatal exception while setting the start state");
        return false;
      }

      try{
        int ret = env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
        if ( (ret < 0) or (lattice_planner_->set_goal(ret) == 0) ) {
          ROS_ERROR_STREAM(name_ << ": Failed to set goal state");
          return false;
        }
      } catch ( SBPL_Exception e ) {
        ROS_ERROR_STREAM(name_ << ": [SBPL] encountered a fatal exception while setting the goal state");
        return false;
      }

      int offOnCount = 0;
      int onOffCount = 0;
      int allCount = 0;
      std::vector<nav2dcell_t> changedcellsV;

      for (unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ix++) {
        for (unsigned int iy = 0; iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++) {

          unsigned char oldCost = env_->GetMapCost(ix,iy);
          unsigned char newCost = costMapCostToSBPLCost_(costmap_ros_->getCostmap()->getCost(ix,iy));

          if ( oldCost == newCost )
            continue;

          allCount++;

          //first case - off cell goes on

          if ( (oldCost != costMapCostToSBPLCost_(costmap_2d::LETHAL_OBSTACLE) and oldCost != costMapCostToSBPLCost_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) and
               (newCost == costMapCostToSBPLCost_(costmap_2d::LETHAL_OBSTACLE) or newCost == costMapCostToSBPLCost_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) ) {
            offOnCount++;
          }

          if ( (oldCost == costMapCostToSBPLCost_(costmap_2d::LETHAL_OBSTACLE) or oldCost == costMapCostToSBPLCost_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) and
               (newCost != costMapCostToSBPLCost_(costmap_2d::LETHAL_OBSTACLE) and newCost != costMapCostToSBPLCost_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
            onOffCount++;
          }
          env_->UpdateCost(ix, iy, costMapCostToSBPLCost_(costmap_ros_->getCostmap()->getCost(ix,iy)));

          nav2dcell_t nav2dcell;
          nav2dcell.x = ix;
          nav2dcell.y = iy;
          changedcellsV.push_back(nav2dcell);
        }
      }

      try{
        if ( not changedcellsV.empty() ) {
          StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
          lattice_planner_->costs_changed(*scq);
          delete scq;
        }

        if ( allCount > force_scratch_limit_ ) 
          lattice_planner_->force_planning_from_scratch();
      } catch ( SBPL_Exception e ) {
        ROS_ERROR_STREAM(name_ << ": [SBPL] failed to update the costmap");
        return false;
      }

      //setting planner parameters
      if ( verbose_ ) {
        ROS_INFO("%s: allocated: %f, init eps: %f",name_.c_str(), allocated_time_,initial_epsilon_);
      }

      lattice_planner_->set_initialsolution_eps(initial_epsilon_);
      lattice_planner_->set_search_mode(false);

      if ( verbose_ )
        ROS_INFO_STREAM(name_ << ": Run planner");
  
      std::vector<int> solution_stateIDs;
      int solution_cost;
      try{
        int ret = lattice_planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
        if ( ret )
          if ( verbose_ ) 
            ROS_INFO_STREAM(name_ << ": Solution is found" );
          else {
            ROS_WARN_STREAM(name_ << ": Solution not found.");
            publishStats_(solution_cost, 0, start, goal);
            return false;
          }
      } catch( SBPL_Exception e ) {
        ROS_ERROR_STREAM(name_ << ": [SBPL] encountered a fatal exception while planning" );
        return false;
      }

      if ( verbose_ ) {
        ROS_INFO("%s: Size of solution: %d", name_.c_str(), (int)solution_stateIDs.size());
      }

      std::vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
      try{
        env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
      } catch ( SBPL_Exception e ) {
        ROS_ERROR_STREAM(name_ << ": [SBPL] encountered a fatal exception while reconstructing the path");
        return false;
      }

      if ( verbose_ )
        ROS_INFO("Plan has %d points.\n", (int)sbpl_path.size());
  
      ros::Time plan_time = ros::Time::now();

      //create a message for the plan
      gui_path.poses.resize(sbpl_path.size());
      gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
      gui_path.header.stamp = plan_time;
      for(unsigned int i=0; i<sbpl_path.size(); i++){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();

        pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
        pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
        pose.pose.position.z = start.pose.position.z;

        tf::Quaternion temp;
        temp.setRPY(0,0,sbpl_path[i].theta);
        pose.pose.orientation.x = temp.getX();
        pose.pose.orientation.y = temp.getY();
        pose.pose.orientation.z = temp.getZ();
        pose.pose.orientation.w = temp.getW();

        plan.push_back(pose);

        gui_path.poses[i].pose.position.x = plan[i].pose.position.x;
        gui_path.poses[i].pose.position.y = plan[i].pose.position.y;
        gui_path.poses[i].pose.position.z = plan[i].pose.position.z;
      }

      publishStats_(solution_cost, sbpl_path.size(), start, goal);
      break;
    }
      
    default: {
      dijkstra_planner_->makePlan(start, goal, plan);

      // Publish the path
      ros::Time plan_time = ros::Time::now();
      
      gui_path.poses.resize(plan.size());
      gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
      gui_path.header.stamp = plan_time;
      gui_path.poses = plan;
      
      break;
    }
  }      
 
  plan_pub_.publish(gui_path);
  
  return true;
}

unsigned char GlobalPlanner::costMapCostToSBPLCost_( unsigned char newcost )
{
  if ( newcost == costmap_2d::LETHAL_OBSTACLE ) {
    return lethal_obstacle_;
  } else if ( newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ) {
    return inscribed_inflated_obstacle_;
  } else if ( newcost == 0 or newcost == costmap_2d::NO_INFORMATION ) {
    return 0;
  } else {
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
  }
}

void GlobalPlanner::publishStats_( int solution_cost, int solution_size,
                                 const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal )
{
  // Fill up statistics and publish
  squirrel_nav_msgs::GlobalPlannerStats stats;
  stats.initial_epsilon = initial_epsilon_;
  stats.plan_to_first_solution = false;
  stats.final_number_of_expands = lattice_planner_->get_n_expands();
  stats.allocated_time = allocated_time_;

  stats.time_to_first_solution = lattice_planner_->get_initial_eps_planning_time();
  stats.actual_time = lattice_planner_->get_final_eps_planning_time();
  stats.number_of_expands_initial_solution = lattice_planner_->get_n_expands_init_solution();
  stats.final_epsilon = lattice_planner_->get_final_epsilon();

  stats.solution_cost = solution_cost;
  stats.path_size = solution_size;
  stats.start = start;
  stats.goal = goal;

  stats_pub_.publish(stats);
}

void GlobalPlanner::updatePlannerCallback_( const std_msgs::Bool::ConstPtr& footprint_on_msg )
{
  if ( footprint_on_msg->data == true ) {
    curr_planner_ = LATTICE;
    ROS_INFO_STREAM(name_ << ": Planning with footprint using a lattice planner.");
  } else {
    curr_planner_ = DIJKSTRA;
    ROS_INFO_STREAM(name_ << ": Planning using standard Dijkstra.");
  }
}

}  // namespace squirrel_navigation

// 
// GlobalPlanner.cpp ends here
