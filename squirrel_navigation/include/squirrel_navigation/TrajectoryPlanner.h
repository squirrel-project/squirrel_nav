// TrajectoryPlanner.h --- 
// 
// Filename: TrajectoryPlanner.h
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Sat Feb  6 16:09:43 2016 (+0100)
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

#ifndef SQUIRREL_NAVIGATION_TRAJECTORYPLANNER_H_
#define SQUIRREL_NAVIGATION_TRAJECTORYPLANNER_H_

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>

#include <angles/angles.h>

#include <cassert>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>

namespace squirrel_navigation {

class TrajectoryPlanner
{
 protected:
  TrajectoryPlanner( void );
  
 public:
  class Profile
  {
   public:
    Profile( void ) : x(0),y(0),yaw(0),vx(0),vy(0),vyaw(0) {};
    ~Profile( void ) {};
    
    double x,y,yaw,vx,vy,vyaw;
  };

  class Pose2D
  {
   public:
    Pose2D( void ) : x(0), y(0), yaw(0), t(0) {};
    virtual ~Pose2D( void ) {};
    
    static inline geometry_msgs::Pose toPoseMsg( const Pose2D& p )
    {
      geometry_msgs::Pose pose;
      pose.position.x = p.x;
      pose.position.y = p.y;
      pose.orientation = tf::createQuaternionMsgFromYaw(p.yaw);
      return pose;
    }
    
    double x,y,yaw,t;
  };
  
  static TrajectoryPlanner* getTrajectory( void );
  static void deleteTrajectory( void );
  
  void setVelocityBounds( double, double );

  void makeTrajectory( const std::vector<geometry_msgs::PoseStamped>& );
  void updateTrajectory( const std::vector<geometry_msgs::PoseStamped>&, size_t );
  
  size_t getNodePose( ros::Time&, geometry_msgs::PoseStamped& ) const;
  
  Profile getProfile( const ros::Time& );

  inline std::vector<Pose2D>* getPoses( void )
  {
    return poses_;
  };
  
  inline bool isActive( void )
  {
    return ( t0_ != nullptr );
  };

  inline void deactivate( void )
  {
    if ( t0_ ) {
      delete t0_;
      t0_ = nullptr;
    }
  };

 public:
  static TrajectoryPlanner* trajectory_ptr_;
  static ros::Time* t0_;
  static std::vector<Pose2D>* poses_;

  double max_linear_vel_, max_angular_vel_;

  std::mutex guard_;

  size_t matchIndex_( double ) const;
  
  inline double timeIncrement_( const Pose2D& p1, const Pose2D& p2 ) const
  {
    double dl,da;

    dl = std::hypot(p1.x-p2.x,p1.y-p2.y);
    da = std::abs(angles::normalize_angle(p1.yaw-p2.yaw));
    
    return 1.25*std::max(dl/max_linear_vel_,da/max_angular_vel_);
  };
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_TRAJECTORYPLANNER_H_ */

// 
// TrajectoryPlanner.h ends here
