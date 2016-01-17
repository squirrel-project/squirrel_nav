// PlannerUpdateHandle.h --- 
// 
// Filename: PlannerUpdateHandle.h
// Description: Relé the update signal for both planners
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Sun Jan 17 22:14:33 2016 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Change Log:
// 
// 
// 
// 
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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// 

// Code:

#ifndef SQUIRREL_NAVIGATION_PLANNERUPDATEHANDLE_H_
#define SQUIRREL_NAVIGATION_PLANNERUPDATEHANDLE_H_

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <squirrel_navigation_msgs/PlannerUpdate.h>

#include <stdexcept>

namespace squirrel_navigation {

class PlannerUpdateHandle
{
 public:
  PlannerUpdateHandle( void );
  virtual ~PlannerUpdateHandle( void );
  
  void spin( double hz=5.0 );
  
 private:
  ros::NodeHandle nh_;
  ros::Publisher update_pub_;
  ros::ServiceServer update_srv_;
  
  bool dispatchPlannerUpdate_( squirrel_navigation_msgs::PlannerUpdate::Request&,
                               squirrel_navigation_msgs::PlannerUpdate::Response& );
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_PLANNERUPDATEHANDLE_H_ */

// 
// PlannerUpdateHandle.h ends here
