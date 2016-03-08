// RobotFootprint.cpp --- 
// 
// Filename: RobotFootprint.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Fri Nov 20 13:57:41 2015 (+0100)
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
//   /*********************************************************************
// *
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
// *   * Neither the name of Willow Garage, Inc. nor the names of its
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
// * Author: Eitan Marder-Eppstein
// *         David V. Lu!!
// *********************************************************************/
// 
//

// Code:

#include "squirrel_navigation/RobotFootprint.h"

namespace squirrel_navigation {

RobotFootprint::RobotFootprint( void )
{
  // Empty
}

RobotFootprint::~RobotFootprint( void )
{
  // Empty
}

void RobotFootprint::updateCurrentFootprint( double robot_x, double robot_y, double robot_yaw,  const std::vector<geometry_msgs::Point>& footprint_spec )
{  
  const double cos_th = std::cos(robot_yaw);
  const double sin_th = std::sin(robot_yaw);

  footprint_.clear();
  footprint_.reserve(footprint_spec.size());
  for (unsigned int i = 0; i < footprint_spec.size(); ++i)  {
    const double px = robot_x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    const double py = robot_y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    footprint_.emplace_back(px,py);  
  }
}

bool RobotFootprint::isInside( double px, double py ) const
{
  return isInsideFootprint(footprint_,CGAL_Point2D(px,py),ckern_);
}

}  // namespace squirrel_navigation

// 
// RobotFootprint.cpp ends here
