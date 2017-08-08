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

#ifndef SQUIRREL_NAVIGATION_CONTROLLER_PID_H_
#define SQUIRREL_NAVIGATION_CONTROLLER_PID_H_

#include "squirrel_navigation/ControllerPIDConfig.h"
#include "squirrel_navigation/utils/controller.h"

#include <ros/time.h>

#include <dynamic_reconfigure/server.h>

#include <ros/publisher.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <memory>
#include <string>

namespace squirrel_navigation {

class ControllerPID : public utils::Controller {
 public:
  class Params {
   public:
    static Params defaultParams();

    std::string global_frame_id;
    double kP_lin, kI_lin, kD_lin;
    double kP_ang, kI_ang, kD_ang;
    bool visualize_topics;
  };

 public:
  ControllerPID() : params_(Params::defaultParams()) { init_ = false; }
  ControllerPID(const Params& params) : params_(params) { init_ = false; }
  virtual ~ControllerPID() {}

  // Reset the controller.
  void initialize(const std::string& name) override;
  void reset(const ros::Time& start) override;

  // Compute the command according to PID control law.
  void computeCommand(
      const ros::Time& stamp, const geometry_msgs::Pose& pose,
      const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& vel,
      const geometry_msgs::Twist& ref_vel,
      geometry_msgs::Twist* twist) override;

  // Parameters' read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  // Reconfigure the parameters.
  void reconfigureCallback(ControllerPIDConfig& config, uint32_t level);

  // Visualize command.
  void publishTwist(
      const geometry_msgs::Pose& actuation_pose,
      const geometry_msgs::Twist& cmd, const ros::Time& stamp) const;

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<ControllerPIDConfig>> dsrv_;

  double errIx_, errIy_, errIa_;
  std::unique_ptr<ros::Time> last_stamp_;

  ros::Publisher cmd_pub_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_CONTROLLER_PID_H_ */
