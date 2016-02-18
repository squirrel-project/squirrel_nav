// TrajectoryPlanner.cpp --- 
// 
// Filename: TrajectoryPlanner.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Sat Feb  6 16:25:41 2016 (+0100)
// Version: 
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
// Copyright (c) 2016, Federico Boniardi
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

#include "squirrel_navigation/TrajectoryPlanner.h"

namespace squirrel_navigation {

TrajectoryPlanner::TrajectoryPlanner( void ) :
    max_linear_vel_(1.0),
    max_angular_vel_(2*M_PI)
{  
  ROS_INFO("squirrel_navigation::TrajectoryPlanner started");
}

TrajectoryPlanner* TrajectoryPlanner::getTrajectory( void )
{
  if ( not trajectory_ptr_ ) {
    trajectory_ptr_ = new TrajectoryPlanner;
    
    if ( not poses_ )
      poses_ = new std::vector<Pose2D>;
  }
  
  return trajectory_ptr_;
}

void TrajectoryPlanner::deleteTrajectory( void )
{
  if ( t0_ )
    delete t0_;
  
  if ( poses_ )
    delete poses_;
    
  if ( trajectory_ptr_ )
    delete trajectory_ptr_;
}

void TrajectoryPlanner::setVelocityBounds( double linear_vel, double angular_vel )
{
  max_linear_vel_ = linear_vel;
  max_angular_vel_ = angular_vel;
}

void TrajectoryPlanner::makeTrajectory( const std::vector<geometry_msgs::PoseStamped>& plan )
{
  const size_t n = plan.size();

  if ( n <= 1 )
    return;

  if ( not t0_ ) 
    t0_ = new ros::Time(ros::Time::now());
  
  poses_->resize(n);
  
  for (size_t i=0; i<n; ++i) {
    (*poses_)[i].x = plan[i].pose.position.x;
    (*poses_)[i].y = plan[i].pose.position.y;
    (*poses_)[i].yaw = tf::getYaw(plan[i].pose.orientation);
    if ( i>0 )
      (*poses_)[i].t = (*poses_)[i-1].t + timeIncrement_((*poses_)[i],(*poses_)[i-1]);
  }
}

void TrajectoryPlanner::updateTrajectory( const std::vector<geometry_msgs::PoseStamped>& plan, size_t init )
{
  const size_t n = plan.size();

  if ( n < 1 )
    return;
  
  if ( init >= poses_->size()-1 )
    return;
  
  std::vector<Pose2D> new_poses(n-2);
  new_poses.insert(new_poses.begin(),poses_->begin(),poses_->begin()+init+1);

  double corr_dx = ((*poses_)[init].x - plan[0].pose.position.x);
  double corr_dy = ((*poses_)[init].y - plan[0].pose.position.y);
      
  for (size_t i=init+1,pi=1; i<init+n-1; ++i,++pi) {    
    double dx = plan[pi].pose.position.x - plan[pi-1].pose.position.x;
    double dy = plan[pi].pose.position.y - plan[pi-1].pose.position.y;
        
    new_poses[i].x = new_poses[i-1].x + dx - std::pow(0.5,pi)*corr_dx;
    new_poses[i].y = new_poses[i-1].y + dy - std::pow(0.5,pi)*corr_dy;
    new_poses[i].yaw = tf::getYaw(plan[pi].pose.orientation);
    new_poses[i].t = new_poses[i-1].t + timeIncrement_(new_poses[i],new_poses[i-1]);;
  }

  delete poses_;
  poses_ = new std::vector<Pose2D>;
  *poses_ = new_poses;
}

size_t TrajectoryPlanner::getNodePose( ros::Time& t, geometry_msgs::PoseStamped& p ) const
{
  double tau = t.toSec()-t0_->toSec();

  size_t i = matchIndex_(tau);
  
  p.pose.position.x = (*poses_)[i].x;
  p.pose.position.y = (*poses_)[i].y;
  p.pose.orientation = tf::createQuaternionMsgFromYaw((*poses_)[i].yaw);

  return i;
}

TrajectoryPlanner::Profile TrajectoryPlanner::getProfile( const ros::Time& t )
{ 
  std::unique_lock<std::mutex> lock(guard_);

  double tau = t.toSec()-t0_->toSec();

  Profile out;
  
  if ( tau < 0.0 ) {
    out.x = poses_->front().x;
    out.y = poses_->front().y;
    out.yaw = poses_->front().yaw;
    return out;
  } else if ( tau >= poses_->back().t ) {
    out.x = poses_->back().x;
    out.y = poses_->back().y;
    out.yaw = poses_->back().yaw;
    return out;
  }

  size_t i = matchIndex_(tau)-1;

  double dt = (*poses_)[i+1].t-(*poses_)[i].t;
  double mu = (tau-(*poses_)[i].t)/dt;
  
  out.x = (*poses_)[i].x + mu * ((*poses_)[i+1].x-(*poses_)[i].x);
  out.y = (*poses_)[i].y + mu * ((*poses_)[i+1].y-(*poses_)[i].y);
  out.yaw = angles::normalize_angle((*poses_)[i].yaw + mu * angles::normalize_angle((*poses_)[i+1].yaw-(*poses_)[i].yaw));
  out.vx = ((*poses_)[i+1].x-(*poses_)[i].x)/dt;
  out.vy = ((*poses_)[i+1].y-(*poses_)[i].y)/dt;
  out.vyaw = angles::normalize_angle((*poses_)[i+1].yaw-(*poses_)[i].yaw)/dt;
            
  return out;
}

size_t TrajectoryPlanner::matchIndex_( double t ) const
{
  if ( t<=0.0 )
    return 0;
  if ( t>poses_->back().t )
    return poses_->size();
  
  size_t i=poses_->size()/2, l=0, u=poses_->size();
  while ( not(t>(*poses_)[i-1].t and t<=(*poses_)[i].t) ) {
    if ( t>(*poses_)[i].t )
      l = i;
    else
      u = i;
    i = (u+l)/2; 
  } 

  return i;
};

TrajectoryPlanner* TrajectoryPlanner::trajectory_ptr_ = nullptr;

ros::Time* TrajectoryPlanner::t0_ = nullptr;

std::vector<TrajectoryPlanner::Pose2D>* TrajectoryPlanner::poses_ = nullptr;

}  // namespace squirrel_navigation

// 
// TrajectoryPlanner.cpp ends here