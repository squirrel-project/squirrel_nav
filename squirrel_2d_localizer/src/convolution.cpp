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

#include "squirrel_2d_localizer/convolution.h"

#include <cmath>
#include <iostream>

namespace squirrel_2d_localizer {

namespace convolution {

void computeGaussianConvolution2d(
    double sigma, double resolution, const Matrix<>& matrix, Matrix<>* output) {
#pragma omp parallel for default(shared)
  for (size_t i = 0; i < matrix.rows(); ++i) {
    Vector<> output_row = Vector<>::Zero(output->row(i).size());
    internal::computeGaussianConvolution1d(
        sigma, resolution, matrix.row(i), &output_row);
    output->row(i) = output_row;
  }
#pragma omp parallel for default(shared)
  for (size_t j = 0; j < matrix.cols(); ++j) {
    Vector<> output_col = Vector<>::Zero(output->col(j).size());
    internal::computeGaussianConvolution1d(
        sigma, resolution, output->col(j), &output_col);
    output->col(j) = output_col;
  }
}

namespace internal {

void computeGaussianConvolution1d(
    double sigma, double resolution, const Vector<>& vector, Vector<>* output) {
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

}  // namespace internal

}  // namespace convolution

}  // namespace squirrel_2d_localizer
