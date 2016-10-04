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

#ifndef SQUIRREL_2D_LOCALIZER_MATH_TYPES_H_
#define SQUIRREL_2D_LOCALIZER_MATH_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace squirrel_2d_localizer {

// clang-format off
template <int N = -1> using Vector = Eigen::Matrix<double, N, 1>;
template <int N = -1, int M = -1> using Matrix = Eigen::Matrix<double, N, M>;

template <int N = -1> using VectorI = Eigen::Matrix<int, N, 1>;
template <int N = -1, int M = -1> using MatrixI = Eigen::Matrix<int, N, M>;
// clang-format on

typedef Eigen::Rotation2D<double> Rotation2d;

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_MATH_TYPES_H_ */
