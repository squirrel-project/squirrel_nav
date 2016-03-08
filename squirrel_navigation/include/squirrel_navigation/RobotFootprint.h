// RobotFootprint.h --- 
// 
// Filename: RobotFootprint.h
// Description: 
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Fri Nov 20 13:57:41 2015 (+0100)
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

#ifndef SQUIRREL_NAVIGATION_FOOTPRINTLAYER_H_
#define SQUIRREL_NAVIGATION_FOOTPRINTLAYER_H_

#include <geometry_msgs/Point.h>

#include "squirrel_navigation/Common.h"

namespace squirrel_navigation {

class RobotFootprint
{
 public:
  
  RobotFootprint( void );
  virtual ~RobotFootprint( void );

  void updateCurrentFootprint( double, double, double, const std::vector<geometry_msgs::Point>& );
  bool isInside( double, double ) const;
  
 private:
  Footprint footprint_;
  
  CGAL_Kernel ckern_;
};

}  // namespace squirrel_navigation

#endif // SQUIRREL_NAVIGATION_FOOTPRINTLAYER_H_

// 
// RobotFootprint.h ends here
