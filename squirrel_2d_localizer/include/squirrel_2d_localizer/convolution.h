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

#ifndef SQUIRREL_2D_LOCALIZER_CONVOLUTION_H_
#define SQUIRREL_2D_LOCALIZER_CONVOLUTION_H_

#include "squirrel_2d_localizer/math_types.h"

namespace squirrel_2d_localizer {

namespace convolution {

void computeGaussianConvolution2d(
    double sigma, double resolution, const Matrix<>& matrix, Matrix<>* output);

namespace internal {

void computeGaussianConvolution1d(
    double sigma, double resolution, const Vector<>& vector,  Vector<>* output);

}  // namespace internal

}  // namespace convolution

}  // namespace squirrel_2d_localizer

#endif /* SQUIRREL_2D_LOCALIZER_CONVOLUTION_H_ */
