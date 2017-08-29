// The MIT License (MIT)
//
// Copyright (c) 2016-2017 Federico Boniardi and Wolfram Burgard
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

 private:
  Params params_;

  Eigen::MatrixXd occupancy_map_;
  Eigen::MatrixXi unknown_space_;
};

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_GRID_MAP_H_ */
