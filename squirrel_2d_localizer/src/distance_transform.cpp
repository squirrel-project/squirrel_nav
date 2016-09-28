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

#include "squirrel_2d_localizer/distance_transform.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace squirrel_2d_localizer {

namespace distance_transform {

// Implementation of:
//  Mazuran M. and Boniardi F. and Burgard W., and Tipaldi G.D.,
//  RTL - Relative Topometric Localizer
void computeSquaredDistanceTransform2d(
    const Matrix<>& f, double resolution, Matrix<>* sq_distance_transform) {
  const size_t rows = f.rows(), cols = f.cols();
  assert(
      rows == sq_distance_transform->rows() &&
      cols == sq_distance_transform->cols());
#pragma omp parallel for default(shared)
  for (size_t c = 0; c < cols; ++c) {
    Vector<> dt_vec;
    internal::computeSquaredDistanceTransform1d(f.col(c), 1.0, &dt_vec);
    sq_distance_transform->col(c) = dt_vec;
  }
#pragma omp parallel for default(shared)
  for (size_t r = 0; r < rows; ++r) {
    Vector<> dt_vec;
    internal::computeSquaredDistanceTransform1d(
        sq_distance_transform->row(r).transpose(), resolution, &dt_vec);
    sq_distance_transform->row(r) = dt_vec;
  }
}

namespace internal {

// Implementation of:
//  Felzenszwalb, P. and Huttenlocher, D., 2004.
//  Distance transforms of sampled functions. Cornell University.
// See Algorithm 1 in (this implementation uses the same variable names):
//  http://www.cs.cornell.edu/~dph/papers/dt.pdf
void computeSquaredDistanceTransform1d(
    const Vector<>& f, double resolution, Vector<>* sq_distance_transform) {
  const double sq_resolution = resolution * resolution;
  const size_t n             = f.rows();
  sq_distance_transform->resize(n);
  Vector<> z(n + 1);
  std::vector<int> v(n);
  int k = 0;
  v[0]  = 0;
  z[0]  = -std::numeric_limits<double>::infinity();
  z[1]  = std::numeric_limits<double>::infinity();
  for (int q = 1, k = 0; q < n; ++q) {
    double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    while (s <= z[k]) {
      k--;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    }
    k++;
    v[k]     = q;
    z[k]     = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }
  for (int q = 0, k = 0; q < n; ++q) {
    while (z[k + 1] < q)
      k++;
    (*sq_distance_transform)[q] =
        sq_resolution * (std::pow(q - v[k], 2) + f[v[k]]);
  }
}

}  // namespace internal

}  // namespace distance_transform

}  // namespace squirrel_2d_localizer
