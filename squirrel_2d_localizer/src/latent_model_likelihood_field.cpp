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

#ifndef SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_CPP_
#define SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_CPP_

#include "squirrel_2d_localizer/latent_model_likelihood_field.h"
#include "squirrel_2d_localizer/convolution.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace squirrel_2d_localizer {

void LatentModelLikelihoodField::initialize(const GridMap& occupancy_gridmap) {
  const size_t h   = occupancy_gridmap.height();
  const size_t w   = occupancy_gridmap.width();
  const double res = occupancy_gridmap.resolution();
  likelihood_cache_ = Matrix<>::Zero(h, w);
  // Compute Gaussian convolution on the map.
  convolution::computeGaussianConvolution2d(
      lmlf_params_.observation_sigma, res, occupancy_gridmap,
      &likelihood_cache_);
  // Add the uniform distribution.
  likelihood_cache_ += lmlf_params_.uniform_hit * Matrix<>::Ones(h, w);
}

double LatentModelLikelihoodField::likelihood(int i, int j) const {
  if (inside(i, j))
    return likelihood_cache_(i, j);
  return lmlf_params_.uniform_hit;
}

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_CPP_ */
