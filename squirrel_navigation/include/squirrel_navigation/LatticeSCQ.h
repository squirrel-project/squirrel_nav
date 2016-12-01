// LatticeSCQ.h ---
//
// Filename: LatticeSCQ.h
// Description: Utility for the global planner
// Author: Federico Boniardi
// Maintainer: boniardi@informatik.uni-freiburg.de
// Created: Sun Jan 17 18:14:17 2016 (+0100)
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

#ifndef SQUIRREL_NAVIGATION_LATTICESCQ_H_
#define SQUIRREL_NAVIGATION_LATTICESCQ_H_

#include <sbpl/headers.h>

#include <vector>

namespace squirrel_navigation {

class LatticeSCQ : public StateChangeQuery {
 public:
  LatticeSCQ(EnvironmentNAVXYTHETALAT*, std::vector<nav2dcell_t> const&);

  virtual std::vector<int> const* getPredecessors(void) const;
  virtual std::vector<int> const* getSuccessors(void) const;

  EnvironmentNAVXYTHETALAT* env_;
  std::vector<nav2dcell_t> const& changedcellsV_;
  mutable std::vector<int> predsOfChangedCells_;
  mutable std::vector<int> succsOfChangedCells_;
};

}  // namespace squirrel_navigation

#endif /* SQUIRREL_NAVIGATION_LATTICESCQ_H_ */
