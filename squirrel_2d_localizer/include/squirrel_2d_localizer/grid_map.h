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

#include <vector>

namespace squirrel_2d_localizer {

class GridMap {
 public:
  class Params {
   public:
    static Params defaultParams();

    double resolution;
    Pose2d origin;
    size_t height;
    size_t width;
  };

 public:
  GridMap() : params_(Params::defaultParams()) {}
  GridMap(const Params& params);
  virtual ~GridMap() {}

  // Initialize the gridmap.
  void initialize(const std::vector<signed char>& data);

  // Rasterization and size utilities.
  void pointToIndices(const EndPoint2d& e, int* i, int* j) const;
  bool inside(int i, int j) const;
  void boundingBox(
      double* min_x, double* max_x, double* min_y, double* max_y) const;

  // Access the gridmap.
  double operator()(int i, int j) const { return occupancy_map_(i, j); };
  double at(int i, int j) const { return occupancy_map_(i, j); }
  operator const Eigen::MatrixXd&() const { return occupancy_map_; }

  // Query uknown free space (for resampling).
  bool unknown(int i, int j) const { return !!unknown_space_(i, j); }

  // Parameters read/write utilities.
  inline const Params& params() const { return params_; }
  inline void setParams(const Params& params) { params_ = params; }
  inline Params& params() { return params_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 protected:
  Params params_;

 private:
  Eigen::MatrixXd occupancy_map_;
  Eigen::MatrixXi unknown_space_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_GRID_MAP_H_ */
