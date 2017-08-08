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

#ifndef SQUIRREL_NAVIGATION_UTILS_SBPL_UTILS_H_
#define SQUIRREL_NAVIGATION_UTILS_SBPL_UTILS_H_

#include <sbpl/headers.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

namespace squirrel_navigation {
namespace sbpl {

typedef EnvironmentNAVXYTHETALAT NavigationEnvironment;

typedef SBPLPlanner Planner;
typedef ARAPlanner ARAstar;

typedef sbpl_2Dpt_t Point;

typedef SBPL_Exception Exception;

typedef EnvNAVXYTHETALAT3Dpt_t Pose;

inline std::vector<Point> footprint(
    const std::vector<geometry_msgs::Point>& ros_footprint) {
  std::vector<Point> sbpl_footprint;
  sbpl_footprint.reserve(ros_footprint.size());
  for (const auto& point32 : ros_footprint) {
    Point point;
    point.x = point32.x;
    point.y = point32.y;
    sbpl_footprint.emplace_back(point);
  }
  return sbpl_footprint;
}

}  // namespace sbpl
}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_UTILS_SBPL_UTILS_H_ */
