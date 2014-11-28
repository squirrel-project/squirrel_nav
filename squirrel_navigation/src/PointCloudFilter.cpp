// PointCloudFilter.h --- 
// 
// Filename: PointCloudFilter.h
// Description: Publish a light PointCloud2 for 2D navigation
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Mon Nov 17 21:31:45 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Wed Nov 26 15:49:26 2014 (+0100)
//           By: Federico Boniardi
//     Update #: 1
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro, ROS Indigo
// 

// Commentary: 
//   Tested on: - ROS Hydro on Ubuntu 12.04
//               - ROS Indigo on Ubuntu 14.04
//   RBGD source: ASUS Xtion pro
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

#include <ros/time.h>

// #include <stdio.h>
#include <sstream>

namespace squirrel_navigation {

PointCloudFilter::PointCloudFilter( void ) :
    private_nh_("~")
{
  private_nh_.param("pointcloud_in", pointcloud_in_topic_, std::string("/cloud_in"));
  private_nh_.param("pointcloud_out", pointcloud_out_topic_, std::string("/cloud_out"));
  private_nh_.param("pointcloud_size", pointcloud_size_, std::string("4500"));

#ifdef DEBUG  
  DEBUG_INFO("pointcloud_in: " << pointcloud_in_topic_);
  DEBUG_INFO("pointcloud_out: " << pointcloud_out_topic_);
  DEBUG_INFO("pointcloud_size: " << pointcloud_size_);
#endif
  
  filter_step_ = getFilterStep();
  seq_ = 0;
  
  sub_ = public_nh_.subscribe(pointcloud_in_topic_, 1, &PointCloudFilter::filterPointCloud, this);

  pointcloud_pub_ = public_nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_out_topic_, 1);
}

PointCloudFilter::~PointCloudFilter( void )
{
  sub_.shutdown();
  pointcloud_pub_.shutdown();
}

void PointCloudFilter::spin( void )
{
  ros::Rate lr(10);
  while ( ros::ok() ) {     
    ros::spinOnce();
    lr.sleep();
  }
}

void PointCloudFilter::filterPointCloud( const sensor_msgs::PointCloud2ConstPtr& pointcloud_in_msg )
{
#ifdef DEBUG
  DEBUG_INFO("filtering pointcloud... ");
#endif
  
  pcl::PointCloud<pcl::PointXYZ> pointcloud_raw;
  pcl::fromROSMsg(*pointcloud_in_msg, pointcloud_raw);

  std::vector<int> nan_filter;
  pcl::removeNaNFromPointCloud(pointcloud_raw, pointcloud_raw, nan_filter);

  pcl::PointCloud<pcl::PointXYZ> pointcloud_filtered;

  for (unsigned int i=0; i<pointcloud_raw.points.size(); i+=filter_step_) {
    pointcloud_filtered.points.push_back(pointcloud_raw[i]);
  }

  sensor_msgs::PointCloud2 pointcloud_out_msg;
  toROSMsg(pointcloud_filtered, pointcloud_out_msg);
  pointcloud_out_msg.header.seq = seq_++;
  pointcloud_out_msg.header.stamp = ros::Time::now();
  pointcloud_out_msg.header.frame_id = pointcloud_in_msg->header.frame_id;

  pointcloud_pub_.publish(pointcloud_out_msg);

#ifdef DEBUG
  DEBUG_INFO("pointcloud published");
#endif
}

int PointCloudFilter::getFilterStep( void )
{
  if ( pointcloud_size_.compare("full") == 0 ) {
    ROS_WARN("chosen 'full pointcloud'. Consider reducing size of the pointcloud.", pointcloud_in_topic_.c_str());
    return 1;
  } else {
    int size;
    std::istringstream(pointcloud_size_) >> size;
    return 300000/std::max(1,size);
  }
}

}  // namespace squirrel_navigation

// 
// PointCloudFilter.cpp ends here
