// LatticeSCQ.cpp ---
//
// Filename: LatticeSCQ.cpp
// Description: Utility for the global planner
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Sun Jan 17 18:18:39 2016 (+0100)
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
// /*********************************************************************
// *
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2008, Willow Garage, Inc.
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of the Willow Garage nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// * Author: Mike Phillips
// *********************************************************************/

#include "squirrel_navigation/LatticeSCQ.h"

namespace squirrel_navigation {

LatticeSCQ::LatticeSCQ(
    EnvironmentNAVXYTHETALAT* env,
    std::vector<nav2dcell_t> const& changedcellsV)
    : env_(env), changedcellsV_(changedcellsV) {
  // Empty
}

std::vector<int> const* LatticeSCQ::getPredecessors(void) const {
  if (predsOfChangedCells_.empty() and (not changedcellsV_.empty()))
    env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
  return &predsOfChangedCells_;
}

std::vector<int> const* LatticeSCQ::getSuccessors(void) const {
  if (succsOfChangedCells_.empty() and (not changedcellsV_.empty()))
    env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
  return &succsOfChangedCells_;
}

}  // namespace squirrel_navigation
