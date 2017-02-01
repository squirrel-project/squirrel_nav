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

#include "squirrel_navigation/PlannerUpdateHandle.h"

namespace squirrel_navigation {

PlannerUpdateHandle::PlannerUpdateHandle(void) {
  update_pub_ = nh_.advertise<std_msgs::Bool>("/plan_with_footprint", 1);
  update_srv_ = nh_.advertiseService(
      "/updatePlanners", &PlannerUpdateHandle::dispatchPlannerUpdate_, this);
}

PlannerUpdateHandle::~PlannerUpdateHandle(void) {
  update_pub_.shutdown();
  update_srv_.shutdown();
  nh_.shutdown();
}

void PlannerUpdateHandle::spin(double hz) {
  ros::Rate lr(hz);
  ros::spin();
}

bool PlannerUpdateHandle::dispatchPlannerUpdate_(
    squirrel_navigation_msgs::PlannerUpdate::Request& req,
    squirrel_navigation_msgs::PlannerUpdate::Response& res) {
  std_msgs::Bool plan_with_footprint;
  plan_with_footprint.data = req.plan_with_footprint.data;
  update_pub_.publish(plan_with_footprint);
  return true;
}

}  // namespace squirrel_navigation
