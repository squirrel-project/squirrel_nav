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

#ifndef SQUIRREL_NAVIGATION_UTILS_MOTION_PLANNER_H_
#define SQUIRREL_NAVIGATION_UTILS_MOTION_PLANNER_H_

#include <ros/time.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <vector>

namespace squirrel_navigation {
namespace utils {

class MotionPlanner {
 public:
  virtual ~MotionPlanner() {}

  virtual void initialize(const std::string& name) = 0;

  virtual void reset(
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      const ros::Time& start) = 0;
  virtual void update(
      const std::vector<geometry_msgs::PoseStamped>& waypoints,
      const ros::Time& stamp) = 0;

  virtual void computeReference(
      const ros::Time& ref_stamp, geometry_msgs::Pose* ref_pose,
      geometry_msgs::Twist* ef_twist) = 0;

  virtual const std::vector<geometry_msgs::PoseStamped>& waypoints() const = 0;

  virtual std::vector<geometry_msgs::PoseStamped> trajectory() const = 0;

 protected:
  bool init_;
};

}  // namespace utils
}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_UTILS_MOTION_PLANNER_H_ */
