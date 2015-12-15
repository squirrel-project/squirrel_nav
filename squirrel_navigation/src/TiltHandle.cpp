// TiltHandle.cpp --- 
// 
// Filename: TiltHandle.cpp
// Description: Check wheter the kinect is tilted or not
// Author: Federico Boniardi
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Thu Mar 12 13:00:04 2015 (+0100)
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

// Copyright (c) 2015, Federico Boniardi
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * redistributions of source code must retain the above copyright notice, this
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

// Code:

#include "squirrel_navigation/TiltHandle.h"

namespace squirrel_navigation {

TiltHandle::TiltHandle( void ) :
    tilt_command_(KINECT_NAVIGATION_ANGLE),
    tilt_moving_(false),
    info_(false)
{
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("state_topic", state_topic_, "/tilt_controller/state");
  pnh.param<std::string>("command_topic", command_topic_, "/tilt_controller/command");
  
  tilt_state_sub_ = nh_.subscribe(state_topic_, 2, &TiltHandle::stateCallback_, this);
  tilt_command_sub_ = nh_.subscribe(command_topic_, 2, &TiltHandle::commandCallback_, this);
}

TiltHandle::~TiltHandle( void )
{
  tilt_state_sub_.shutdown();
  tilt_command_sub_.shutdown();
}

bool TiltHandle::gotMotionCommand( void )
{
  return (std::abs(KINECT_NAVIGATION_ANGLE - tilt_command_) > 1e-3);
}

bool TiltHandle::isMoving( void )
{
  return tilt_moving_;
}

void TiltHandle::printROSMsg( const char* msg )
{
  if ( !info_ ) {
    ROS_INFO("%s: Kinect has been tilted. %s", ros::this_node::getName().c_str(), msg);
    info_ = true;
  } else {
    return;
  }
}

void TiltHandle::stateCallback_( const dynamixel_msgs::JointState::ConstPtr& tilt_state_msg )
{
  tilt_moving_ = tilt_state_msg->is_moving;
}

void TiltHandle::commandCallback_( const std_msgs::Float64::ConstPtr& tilt_cmd_msg )
{
  tilt_command_ = tilt_cmd_msg->data;
  info_ = false;
}

}  // namespace squirrel_navigation

// 
// TiltHandle.cpp ends here
