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
//
//

// Code:

#ifndef SQUIRREL_NAVIGATION_COMMON_H_
#define SQUIRREL_NAVIGATION_COMMON_H_

#include <csignal>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>

#include <tf/tf.h>

#include <angles/angles.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace squirrel_navigation {

static sig_atomic_t _SIGINT_caught = 0;

static const double PI = 3.14159265358979;

static const double TRANSFORM_TIMEOUT = 0.5;

static const int VOXEL_BITS = 16;

static const double TOLL = 1e-3;

static void interruptCallback(int sig) { _SIGINT_caught = 1; };

inline double linearDistance(
    const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
};

inline double angularDistance(
    const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2) {
  double da = angles::normalize_angle(tf::getYaw(q1) - tf::getYaw(q2));
  return std::abs(da);
};

// Just a l2 distance on SE(2)
inline double specialEuclideanDistance(
    const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
  double da = angles::normalize_angle(
      tf::getYaw(p1.orientation) - tf::getYaw(p2.orientation));
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  return std::sqrt(da * da + dx * dx + dy * dy);
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel CGAL_Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel::Point_2
    CGAL_Point2D;
typedef std::vector<
    CGAL::Exact_predicates_inexact_constructions_kernel::Point_2>
    Footprint;

inline std::vector<geometry_msgs::Point> toPointVector(const Footprint& pgn) {
  const size_t n = pgn.size();

  std::vector<geometry_msgs::Point> out(n);
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::Point new_pt;
    new_pt.x = pgn[i].x();
    new_pt.y = pgn[i].y();
    out[i]   = new_pt;
  }

  return out;
};

inline bool isInsideFootprint(
    const Footprint& footprint, const CGAL_Point2D& pt, const CGAL_Kernel& k) {
  return (
      CGAL::bounded_side_2(&footprint.front(), &footprint.back(), pt, k) !=
      CGAL::ON_UNBOUNDED_SIDE);
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_COMMON_H_ */
