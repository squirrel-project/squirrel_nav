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

#include "squirrel_2d_localizer/convolution.h"

#include <cmath>
#include <iostream>

namespace squirrel_2d_localizer {
namespace convolution {

void computeGaussianConvolution2d(
    double sigma, double resolution, const Eigen::MatrixXd& matrix,
    Eigen::MatrixXd* output) {
#pragma omp parallel for default(shared)
  for (size_t i = 0; i < matrix.rows(); ++i) {
    Eigen::VectorXd output_row = Eigen::VectorXd::Zero(output->row(i).size());
    __internal::computeGaussianConvolution1d(
        sigma, resolution, matrix.row(i), &output_row);
    output->row(i) = output_row;
  }
#pragma omp parallel for default(shared)
  for (size_t j = 0; j < matrix.cols(); ++j) {
    Eigen::VectorXd output_col = Eigen::VectorXd::Zero(output->col(j).size());
    __internal::computeGaussianConvolution1d(
        sigma, resolution, output->col(j), &output_col);
    output->col(j) = output_col;
  }
}

namespace __internal {

void computeGaussianConvolution1d(
    double sigma, double resolution, const Eigen::VectorXd& vector,
    Eigen::VectorXd* output) {
  const int vsize = vector.size();
  if (vsize < 1)
    return;
  const double pixel_sigma   = sigma / resolution;
  const double normalization = 1 / std::sqrt(2 * pixel_sigma * M_PI);
  const int radius           = 5 * pixel_sigma;
  for (int i = 0; i < vsize; ++i) {
    const int min_index = std::max(0, i - radius);
    const int max_index = std::min(vsize, i + radius);
    for (int j = min_index; j < max_index; ++j) {
      const double d = (i - j) / pixel_sigma;
      (*output)(j) += vector(i) * normalization * std::exp(-0.5 * d * d);
    }
  }
}

}  // namespace __internal
}  // namespace convolution
}  // namespace squirrel_2d_localizer
