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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "squirrel_navigation/RobotMultiFootprint.h"

namespace squirrel_navigation {

RobotMultiFootprint::RobotMultiFootprint(void) {
  // Empty
}

RobotMultiFootprint::~RobotMultiFootprint(void) {
  std::for_each(
      footprints_pubs_.begin(), footprints_pubs_.end(),
      [](ros::Publisher& pub) { pub.shutdown(); });
}

void RobotMultiFootprint::updateCurrentMultiFootprint(
    double robot_x, double robot_y, double robot_yaw) {
  const size_t n = footprints_specs_.size();

  ros::Time now = ros::Time::now();

  geometry_msgs::PolygonStamped footprint_msg;
  for (size_t i = 0; i < n; ++i) {
    footprint_msg.header.frame_id = "/map";
    footprint_msg.header.stamp    = now;
    footprint_msg.polygon.points.clear();

    const double cos_th = cos(robot_yaw);
    const double sin_th = sin(robot_yaw);

    footprints_[i].clear();
    footprints_[i].reserve(footprints_specs_[i].size());

    for (size_t j = 0; j < footprints_specs_[i].size(); ++j) {
      geometry_msgs::Point32 new_pt;
      new_pt.x = robot_x + (footprints_specs_[i][j].x * cos_th -
                            footprints_specs_[i][j].y * sin_th);
      new_pt.y = robot_y + (footprints_specs_[i][j].x * sin_th +
                            footprints_specs_[i][j].y * cos_th);
      footprint_msg.polygon.points.push_back(new_pt);
      footprints_[i].emplace_back(new_pt.x, new_pt.y);
    }

    footprints_pubs_[i].publish(footprint_msg);
  }
}

bool RobotMultiFootprint::isInside(
    double px, double py, unsigned int layer) const {
  return isInsideFootprint(footprints_[layer], CGAL_Point2D(px, py), ckern_);
}

void RobotMultiFootprint::onInitialize(void) {
  ros::NodeHandle pnh("~");

  XmlRpc::XmlRpcValue footprints_list, ids_list;
  pnh.getParam("footprints", footprints_list);
  pnh.getParam("footprints_id", ids_list);

  ROS_ASSERT(footprints_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(ids_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(footprints_list.size() == ids_list.size());

  const size_t n = footprints_list.size();

  // Looping on layers
  footprints_pubs_.resize(n);
  footprints_specs_.resize(n);
  in_radii_.resize(n);
  circ_radii_.resize(n);

  for (size_t i = 0; i < n; ++i) {
    ROS_ASSERT(footprints_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(ids_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);

    footprints_pubs_[i] = pnh.advertise<geometry_msgs::PolygonStamped>(
        std::string(ids_list[i]) + "_footprint", 1);

    // Looping on polygon
    footprints_specs_[i].reserve(footprints_list[i].size());
    for (size_t j = 0; j < footprints_list[i].size(); ++i) {
      ROS_ASSERT(
          (footprints_list[i][j].getType() ==
           XmlRpc::XmlRpcValue::TypeArray) and
          (footprints_list[i][j].size() == 2));
      geometry_msgs::Point new_pt;
      new_pt.x = footprints_list[i][j][0];
      new_pt.y = footprints_list[i][j][1];
      footprints_specs_[i].push_back(new_pt);
    }

    costmap_2d::calculateMinAndMaxDistances(
        footprints_specs_[i], in_radii_[i], circ_radii_[i]);
  }

  footprints_.resize(n);
}

}  // namespace squirrel_navigation
