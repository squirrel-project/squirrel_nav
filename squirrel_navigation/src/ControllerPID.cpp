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

#include "squirrel_navigation/ControllerPID.h"

namespace squirrel_navigation {

ControllerPID::ControllerPID(void)
    : t_(nullptr), I_err_x_(0.0), I_err_y_(0.0), I_err_yaw_(0.0) {
  // Empty
}

ControllerPID::~ControllerPID(void) {
  if (t_)
    delete t_;
}

void ControllerPID::setGains(const ControllerPID::Gain& gains) {
  K_.Pxy  = gains.Pxy;
  K_.Pyaw = gains.Pyaw;
  K_.Ixy  = gains.Ixy;
  K_.Iyaw = gains.Iyaw;
  K_.Dxy  = gains.Dxy;
  K_.Dyaw = gains.Dyaw;
}

void ControllerPID::computeCommands(
    const TrajectoryPlanner::Profile& ref,
    const TrajectoryPlanner::Profile& odom, double t, double* u) {
  const double dt = t - *t_;

  const double err_x   = ref.x - odom.x;
  const double err_y   = ref.y - odom.y;
  const double err_yaw = angles::normalize_angle(ref.yaw - odom.yaw);

  I_err_x_ += err_x * dt;
  I_err_y_ += err_y * dt;
  I_err_yaw_ = angles::normalize_angle(I_err_yaw_ + err_yaw * dt);

  u[0] = K_.Pxy * err_x + K_.Ixy * I_err_x_ + K_.Dxy * (ref.vx - odom.vx);
  u[1] = K_.Pxy * err_y + K_.Ixy * I_err_y_ + K_.Dxy * (ref.vy - odom.vy);
  u[2] = K_.Pyaw * err_yaw + K_.Iyaw * I_err_yaw_ +
         K_.Dyaw * (ref.vyaw - odom.vyaw);

  *t_ = t;
}

void ControllerPID::activate(double t0) {
  if (not t_)
    t_ = new double(t0);
  else
    *t_ = t0;

  I_err_x_   = 0.0;
  I_err_y_   = 0.0;
  I_err_yaw_ = 0.0;
}

void ControllerPID::deactivate(void) {
  if (t_)
    delete t_;
  t_         = nullptr;
  I_err_x_   = 0.0;
  I_err_y_   = 0.0;
  I_err_yaw_ = 0.0;
}

}  // namespace squirrel_navigation
