// Copyright (c) 2015, Federico Boniardi
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

#ifndef SQUIRREL_NAVIGATION_JOINTHANDLE_H_
#define SQUIRREL_NAVIGATION_JOINTHANDLE_H_

#include <ros/ros.h>

#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "squirrel_navigation/Common.h"

namespace squirrel_navigation {

class JointHandle {
 public:
  JointHandle(void);
  JointHandle(const std::string&);
  virtual ~JointHandle(void);

  inline bool skipData(void) {
    if (not use_navigation_angle_)
      return false;
    else
      return (isMoving_() or gotMotionCommand_());
  };

 private:
  ros::NodeHandle nh_;
  ros::Subscriber command_sub_, state_sub_;

  double command_, cur_angle_;
  bool moving_, info_;
  std::string name_;

  // Parameters
  std::string command_topic_, state_topic_;
  double navigation_angle_;
  bool use_navigation_angle_, verbose_;

  void stateCallback_(const dynamixel_msgs::JointState::ConstPtr&);
  void commandCallback_(const std_msgs::Float64::ConstPtr&);

  inline bool gotMotionCommand_(void) const {
    return (std::abs(navigation_angle_ - cur_angle_) > 1e-3) or command_;
  };

  inline bool isMoving_(void) const { return moving_; };
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_JOINTHANDLE_H_ */
