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

#ifndef SQUIRREL_NAVIGATION_ROBOTMULTIFOOTPRINT_H_
#define SQUIRREL_NAVIGATION_ROBOTMULTIFOOTPRINT_H_

#include <ros/ros.h>

#include <costmap_2d/footprint.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

#include "squirrel_navigation/Common.h"

#include <algorithm>
#include <string>

namespace squirrel_navigation {

class RobotMultiFootprint {
 public:
  RobotMultiFootprint(void);
  virtual ~RobotMultiFootprint(void);

  void onInitialize(void);
  void updateCurrentMultiFootprint(double, double, double);
  bool isInside(double, double, unsigned int) const;

  inline double inscribedRadius(unsigned int l) const { return in_radii_[l]; };
  inline double circumscribedRadius(unsigned int l) const {
    return circ_radii_[l];
  };

 private:
  std::vector<Footprint> footprints_;

  std::vector<ros::Publisher> footprints_pubs_;
  std::vector<std::vector<geometry_msgs::Point>> footprints_specs_;
  std::vector<double> in_radii_, circ_radii_;

  CGAL_Kernel ckern_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_ROBOTMULTIFOOTPRINT_H_ */
