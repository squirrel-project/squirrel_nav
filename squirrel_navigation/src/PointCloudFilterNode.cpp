// PointCloudFilterNode.cpp --- 
// 
// Filename: PointCloudFilterNode.cpp
// Description: Publish a light PointCloud2 for 2D navigation
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Mon Nov 17 21:31:45 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Wed Nov 26 15:52:44 2014 (+0100)
//           By: Federico Boniardi
//     Update #: 1
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro, ROS Indigo 
// 

// Commentary: 
//    Tested on: - ROS Hydro on Ubuntu 12.04
//               - ROS Indigo on Ubuntu 14.04
//    RGBD source: ASUS Xtion pro
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
// * Neither the name of the {organization} nor the names of its
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

#include "squirrel_navigation/PointCloudFilter.h"

#include <ros/ros.h>

using squirrel_navigation::PointCloudFilter;

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "pointcloud_filter");
  PointCloudFilter pf;
  pf.spin();
  return 0;
}

// 
// PointCloudFilterNode.cpp ends here
