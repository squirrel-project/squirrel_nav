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

#include "squirrel_navigation/CostmapUpdateHandle.h"

namespace squirrel_navigation {

CostmapUpdateHandle::CostmapUpdateHandle(void) : update_(true) {
  // Empty
}

CostmapUpdateHandle* CostmapUpdateHandle::getHandle(void) {
  if (not update_handle_) {
    update_handle_ = new CostmapUpdateHandle;
    update_handle_->init_();
  }

  return update_handle_;
}

void CostmapUpdateHandle::releaseHandle(void) {
  if (update_handle_)
    delete update_handle_;
}

void CostmapUpdateHandle::init_(void) {
  ros::NodeHandle pnh("~");
  sub_ = pnh.subscribe(
      "/update_costmap", 1, &CostmapUpdateHandle::updateCallback_, this);
}

void CostmapUpdateHandle::updateCallback_(
    const std_msgs::Bool::ConstPtr& update_msg) {
  update_ = update_msg->data;
}

CostmapUpdateHandle* CostmapUpdateHandle::update_handle_ = nullptr;

}  // namespace squirrel_navigation
