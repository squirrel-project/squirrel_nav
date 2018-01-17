// Copyright (c) 2017, Federico Boniardi and Wolfram Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the University of Freiburg nor the names of
//   its contributors may be used to endorse or promote products
//   derived from this software without specific prior written
//   permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SQUIRREL_POINTCLOUD_FILTER_FILTERS_END_EFFECTOR_FILTER_H_
#define SQUIRREL_POINTCLOUD_FILTER_FILTERS_END_EFFECTOR_FILTER_H_

#include "squirrel_pointcloud_filters/filters/EndEffectorFilterConfig.h"
#include "squirrel_pointcloud_filters/filters/filters.h"

#include <std_msgs/Float32.h>

#include <dynamic_reconfigure/server.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>

namespace squirrel_pointcloud_filters {

class EndEffectorFilter : public Filter {
 public:
  EndEffectorFilter() : dsrv_(nullptr), initialized_(false) {}
  EndEffectorFilter(const std::string& name);
  virtual ~EndEffectorFilter() {}

  void initialize(const std::string& name) override;
  void apply(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud) const override;

 private:
  void reconfigureCallback(EndEffectorFilterConfig& config, uint32_t level);
  void endEffectorCallback(const std_msgs::Float32::ConstPtr& radius);
  
 private:
  std::unique_ptr<dynamic_reconfigure::Server<EndEffectorFilterConfig>> dsrv_;

  std::string end_effector_frame_;
  bool enabled_;
  bool const_end_effector_dim_;
  std::string end_effector_topic_;
  double sq_radius_;
  
  bool initialized_;

  ros::Subscriber sub_;

  tf::TransformListener tfl_;
  
  mutable std::mutex mtx_;
};

}  // namespace squirrel_pointcloud_filters

#endif /* SQUIRREL_POINTCLOUD_FILTER_FILTERS_END_EFFECTOR_FILTER_H_ */
