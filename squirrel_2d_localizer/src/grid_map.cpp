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

#include "squirrel_2d_localizer/grid_map.h"

#include <cmath>

namespace squirrel_2d_localizer {

GridMap::GridMap(const Params& params) : params_(params) {
  const size_t h = params_.height;
  const size_t w = params_.width;
  occupancy_map_.resize(h, w);
  unknown_space_.resize(h, w);
}

void GridMap::initialize(const std::vector<signed char>& data) {
  assert(params_.height * params_.width == data.size());
  const double normalizer = 1. / 100.;
#pragma omp parallel for default(shared)
  for (size_t px = 0; px < data.size(); ++px) {
    const size_t i        = params_.height - 1 - px / params_.width;
    const size_t j        = px % params_.width;
    const int pixel_value = static_cast<int>(data[px]);
    occupancy_map_(i, j) = pixel_value >= 0 ? pixel_value * normalizer : 0.;
    unknown_space_(i, j) = pixel_value >= 0 ? 0 : 1;
  }
}

void GridMap::pointToIndices(const EndPoint2d& e, int* i, int* j) const {
  const double x = e[0] - params_.origin[0];
  const double y = e[1] - params_.origin[1];
  *i             = params_.height - std::floor(y / params_.resolution) - 1;
  *j             = std::floor(x / params_.resolution);
}

bool GridMap::inside(int i, int j) const {
  return (i >= 0 && i < params_.height) && (j >= 0 && j < params_.width);
}

void GridMap::boundingBox(
    double* min_x, double* max_x, double* min_y, double* max_y) const {
  *min_x = params_.origin[0];
  *max_x = params_.origin[0] + params_.resolution * params_.width;
  *min_y = params_.origin[1];
  *max_y = params_.origin[1] + params_.resolution * params_.height;
}

GridMap::Params GridMap::Params::defaultParams() {
  Params params;
  params.resolution = 0.05;
  params.origin     = Pose2d(-10., -10., 0.);
  params.height     = 400;
  params.width      = 400;
  return params;
}

}  // namespace squirrel_2d_localizer
