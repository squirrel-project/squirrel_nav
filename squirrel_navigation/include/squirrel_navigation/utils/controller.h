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
// OF THE POSSIBILITY OF SUCH DAMAGE

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
