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

#ifndef SQUIRREL_NAVIGATION_CONTROLLER_PID_H_
#define SQUIRREL_NAVIGATION_CONTROLLER_PID_H_

#include <dynamic_reconfigure/server.h>

#include <squirrel_navigation/ControllerPIDConfig.h>

#include <memory>
#include <string>

namespace squirrel_navigation {

class ControllerPID {
 public:
  class Params {
   public:
    static Params defaultParams();

    double kP_lin, kI_lin, kD_lin;
    double kP_ang, kI_ang, kD_ang;
  };

 public:
  ControllerPID() : params_(Params::defaultParams()) {}
  ControllerPID(const Params& params) : params_(params) {}
  virtual ~ControllerPID() {}

  // Reset the controller.
  void intialize(const std::string& name);
  void reset();

  // Compute the command according to PID control law.
  void computeCommand(
      const ros::Time& stamp, const geometry_msgs::Pose& pose,
      const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& vel,
      const geometry_msgs::Twist& ref_vel, geometry_msgs::Twist* twist);

  // Set/get initial time.
  const ros::Time* start() const { return start_.get(); }
  void setStart(const ros::Time& t) { start_.reset(new ros::Time(t)); }

  // Parameters' read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

 private:
  void reconfigureCallback(ControllerPIDConfig& config, uint32_t level);

 private:
  Params params_;
  std::unique_ptr<dynamic_reconfigure::Server<ControllerPIDConfig>> dsrv_;

  double errIx_, errIy_, errIa_;
  std::unique_ptr<ros::Time> last_stamp_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_CONTROLLER_PID_H_ */
