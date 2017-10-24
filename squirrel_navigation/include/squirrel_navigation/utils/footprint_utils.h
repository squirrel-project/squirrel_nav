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

#ifndef SQUIRREL_NAVIGATION_FOOTPRINT_UTILS_H_
#define SQUIRREL_NAVIGATION_FOOTPRINT_UTILS_H_

#include "squirrel_navigation/utils/math_utils.h"

#include <costmap_2d/footprint.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

#include <vector>

namespace squirrel_navigation {
namespace footprint {

inline std::vector<geometry_msgs::Point> closedPolygon(
    const std::vector<geometry_msgs::Point>& open_polygon) {
  const int npoints = open_polygon.size();
  if (npoints <= 1)
    return open_polygon;
  std::vector<geometry_msgs::Point> output;
  output.reserve(npoints);
  for (int i = 0; i <= npoints; ++i)
    output.emplace_back(open_polygon[i % npoints]);
  return output;
}

inline bool equal(
    const std::vector<geometry_msgs::Point>& footprint1,
    const std::vector<geometry_msgs::Point>& footprint2) {
  if (footprint1.size() != footprint2.size())
    return false;
  for (unsigned int i = 0; i < footprint1.size(); ++i)
    if (math::linearDistance2D(footprint1[i], footprint2[i]) > 1e-8)
      return false;
  return true;
}

inline bool equal(
    const geometry_msgs::PolygonStamped& footprint_msg,
    const std::vector<geometry_msgs::Point>& footprint_spec) {
  return equal(
      costmap_2d::toPointVector(footprint_msg.polygon), footprint_spec);
}

}  // namespace footprint
}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_FOOTPRINT_UTILS_H_ */
