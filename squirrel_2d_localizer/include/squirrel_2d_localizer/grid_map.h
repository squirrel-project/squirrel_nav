// Copyright (c) 2016-2017, Federico Boniardi and Wolfram Burgard
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

#ifndef SQUIRREL_2D_LOCALIZER_GRID_MAP_H_
#define SQUIRREL_2D_LOCALIZER_GRID_MAP_H_

#include "squirrel_2d_localizer/endpoint_types.h"
#include "squirrel_2d_localizer/math_types.h"
#include "squirrel_2d_localizer/se2_types.h"

#include <memory>
#include <vector>

namespace squirrel_2d_localizer {

class GridMap {
 public:
  typedef std::unique_ptr<GridMap> Ptr;
  typedef std::unique_ptr<GridMap const> ConstPtr;

  struct Params {
    double resolution;
    Pose2d origin;
    size_t height;
    size_t width;
  };

 public:
  GridMap() { setDefaultParams(); }
  GridMap(const Params& map_params);
  virtual ~GridMap() {}

  void initialize(const std::vector<signed char>& data);

  void pointToIndices(const EndPoint2d& e, int* i, int* j) const;
  bool inside(int i, int j) const;

  size_t height() const { return map_params_.height; }
  size_t width() const { return map_params_.width; }
  double resolution() const { return map_params_.resolution; }

  double operator()(int i, int j) const { return occupancy_map_(i, j); };
  operator const Matrix<>&() const { return occupancy_map_; }

  inline Params& params() { return map_params_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
 private:
  inline void setDefaultParams() {
    map_params_.resolution = 0.05;
    map_params_.origin     = Pose2d(-10., -10., 0.);
    map_params_.height     = 400;
    map_params_.width      = 400;
  }

 private:
  Matrix<> occupancy_map_;

  Params map_params_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_GRID_MAP_H_ */
