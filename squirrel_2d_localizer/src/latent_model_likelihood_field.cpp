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

#ifndef SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_CPP_
#define SQUIRREL_2D_LOCALIZER_LATENT_MODEL_LIKELIHOOD_FIELD_CPP_

#include "squirrel_2d_localizer/latent_model_likelihood_field.h"
#include "squirrel_2d_localizer/convolution.h"

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
