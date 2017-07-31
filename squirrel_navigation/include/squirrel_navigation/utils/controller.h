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

#ifndef SQUIRREL_NAVIGATION_UTILS_CONTROLLER_H_
#define SQUIRREL_NAVIGATION_UTILS_CONTROLLER_H_

#include <ros/time.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <string>

namespace squirrel_navigation {
namespace utils {

class Controller {
 public:
  virtual ~Controller() {}

  virtual void initialize(const std::string& name) = 0;
  virtual void reset(const ros::Time& start) = 0;

  virtual void computeCommand(
      const ros::Time& stamp, const geometry_msgs::Pose& pose,
      const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& vel,
      const geometry_msgs::Twist& ref_vel, geometry_msgs::Twist* twist) = 0;

 protected:
  bool init_;
};

}  // namespace utils
}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_UTILS_CONTROLLER_H_ */
