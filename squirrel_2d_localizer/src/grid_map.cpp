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

#include "squirrel_2d_localizer/grid_map.h"

namespace squirrel_2d_localizer {

GridMap::GridMap(const Params& map_params) : map_params_(map_params) {
  const size_t h = map_params_.height;
  const size_t w = map_params_.width;
  occupancy_map_.resize(h, w);
}

void GridMap::initialize(const std::vector<signed char>& data) {
  assert(map_params_.height * map_params_.width == data.size());
  const double normalizer = 1. / 100.;
#pragma omp parallel for default(shared)
  for (size_t px = 0; px < data.size(); ++px) {
    const size_t i        = map_params_.height - 1 - px / map_params_.width;
    const size_t j        = px % map_params_.width;
    const int pixel_value = static_cast<int>(data[px]);
    occupancy_map_(i, j) = pixel_value >= 0 ? pixel_value * normalizer : 0.;
  }
}

void GridMap::pointToIndices(const EndPoint2d& e, int* i, int* j) const {
  const double x = e[0] - map_params_.origin[0];
  const double y = e[1] - map_params_.origin[1];
  *i = map_params_.height - std::floor(y / map_params_.resolution) - 1;
  *j = std::floor(x / map_params_.resolution);
}

bool GridMap::inside(int i, int j) const {
  return (i >= 0 && i < map_params_.height) &&
         (j >= 0 && j < map_params_.width);
}

}  // namespace squirrel_2d_localizer
