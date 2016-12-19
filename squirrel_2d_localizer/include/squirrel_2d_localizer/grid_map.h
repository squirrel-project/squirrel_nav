// The MIT License (MIT)
//
// Copyright (c) 2016 Federico Boniardi
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
