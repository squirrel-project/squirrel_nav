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

#ifndef SQUIRREL_POINTCLOUD_FILTER_FILTERS_ARM_FILTER_H_
#define SQUIRREL_POINTCLOUD_FILTER_FILTERS_ARM_FILTER_H_

#include "squirrel_pointcloud_filter/ArmFilterConfig.h"
#include "squirrel_pointcloud_filter/filters/filter.h"

#include <geometry_msgs/Point.h>

#include <dynamic_reconfigure/server.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace squirrel_pointcloud_filter {

class ArmFilter : public Filter {
  using Segment = std::tuple<tf::Vector3, tf::Vector3, float>;

 public:
  ArmFilter() : dsrv_(nullptr), initialized_(false) {}
  ArmFilter(const std::string& name);
  virtual ~ArmFilter() {}

  void initialize(const std::string& name) override;
  void apply(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud) const override;

 public:
  static const std::string tag;
  
 private:
  void reconfigureCallback(ArmFilterConfig& config, uint32_t level);

  bool getJointsSegments(std::vector<Segment>* segments) const;
  bool inCylinder(
      const Segment& segment, double radius,
      const pcl::PointXYZ& query_point) const;

 private:
  std::unique_ptr<dynamic_reconfigure::Server<ArmFilterConfig>> dsrv_;

  std::vector<std::string> joint_chain_;
  std::vector<double> segments_sq_radii_;

  tf::TransformListener tfl_;

  bool enabled_;

  bool initialized_;
};

}  // namespace squirrel_pointcloud_filter

#endif /* SQUIRREL_POINTCLOUD_FILTER_FILTERS_ARM_FILTER_H_ */
