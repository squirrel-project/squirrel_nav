// ControllerPD.cpp --- 
// 
// Filename: ControllerPD.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Sat Feb  6 19:21:25 2016 (+0100)
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

#include "squirrel_navigation/ControllerPD.h"

namespace squirrel_navigation {

ControllerPD::ControllerPD( void ) :
    toll_(0.05)
{
  // Empty
}

ControllerPD::~ControllerPD( void )
{
  // Empty
}

void ControllerPD::setGains( const ControllerPD::Gain& gains )
{
  K_.Pxy = gains.Pxy;
  K_.Pyaw = gains.Pyaw;
  K_.Dxy = gains.Dxy;
  K_.Dyaw = gains.Dyaw;
}

void ControllerPD::computeCommands( const TrajectoryPlanner::Profile& ref, const TrajectoryPlanner::Profile& odom, double* u )
{
  double err_x = ref.x-odom.x > 0 ? std::max(0.0,ref.x-odom.x-toll_) : std::min(0.0,ref.x-odom.x+toll_);
  double err_y = ref.y-odom.y > 0 ? std::max(0.0,ref.y-odom.y-toll_) : std::min(0.0,ref.y-odom.y+toll_);
   
  u[0] = K_.Pxy * err_x + K_.Dxy * (ref.vx - odom.vx);
  u[1] = K_.Pxy * err_x + K_.Dxy * (ref.vy - odom.vy);
  u[2] = K_.Pyaw * angles::normalize_angle(ref.yaw - odom.yaw) + K_.Dyaw * (ref.vyaw - odom.vyaw);
}
   
}  // namespace squirrel_navigation

// 
// ControllerPD.cpp ends here
