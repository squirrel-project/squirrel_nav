// The MIT License (MIT)
//
// Copyright (c) 2017 Federico Boniardi and Wolfram Burgard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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

 protected:
  bool init_;
};

}  // namespace utils
}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_UTILS_MOTION_PLANNER_H_ */
