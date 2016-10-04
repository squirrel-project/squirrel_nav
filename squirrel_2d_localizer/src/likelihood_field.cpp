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

#include "squirrel_2d_localizer/likelihood_field.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace squirrel_2d_localizer {

void LikelihoodField::initialize(const GridMap& map) {
  const size_t w   = map.width();
  const size_t h   = map.height();
  const double res = map.resolution();
  Matrix<> obstacles_indicator(h, w), sq_edt(h, w);
#pragma omp parallel for schedule(dynamic, 1) collapse(2)
  for (size_t i = 0; i < h; ++i)
    for (size_t j = 0; j < w; ++j)
      obstacles_indicator(i, j) =
          map(i, j) ? 0. : std::numeric_limits<double>::max();
  distance_transform::computeSquaredDistanceTransform2d(
      obstacles_indicator, res, &sq_edt);
  likelihood_cache_.resize(h, w);
  sq_saturation_distance_ =
      lf_params_.saturation_distance * lf_params_.saturation_distance;
#pragma omp parallel for schedule(dynamic, 1) collapse(2)
  for (size_t i = 0; i < h; ++i)
    for (size_t j = 0; j < w; ++j) {
      const double sq_saturated_edt =
          std::min(sq_edt(i, j), sq_saturation_distance_);
      likelihood_cache_(i, j) =
          std::exp(-lf_params_.observation_sigma * sq_saturated_edt);
    }
}

double LikelihoodField::likelihood(int i, int j) const {
  if (inside(i, j))
    return likelihood_cache_(i, j);
  else
    return std::exp(-lf_params_.observation_sigma * sq_saturation_distance_);
}

}  // namespace squirrel_2d_localizer
