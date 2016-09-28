// Lextor - Lexicographic Teach Optimize and Repeat
//
// Copyright (C) 2015-2016 Mladen Mazuran, Wolfram Burgard and
// Gian Diego Tipaldi
//
// This file is part of Lextor. The code in external/csmstrip is a modified
// version of the C(anonical) Scan Matcher (http://purl.org/censi/2007/csm).
// Copyright (C) 2007-2015 Andrea Censi
//
// Lextor is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Lextor is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Lextor.  If not, see <http://www.gnu.org/licenses/>.

#ifndef CSMSTRIP_DEPS_H_
#define CSMSTRIP_DEPS_H_

#include <Eigen/Core>
#include <Eigen/LU>
#include "scalar.h"

/* #define CSMSTRIP_DEBUG */

namespace csm {

#ifdef CSMSTRIP_DEBUG
void sm_debug(const char* msg, ...);
void sm_error(const char* msg, ...);
void sm_info(const char* msg, ...);
#else
#define sm_debug(...) \
  do {                \
  } while (0)
#define sm_error(...) \
  do {                \
  } while (0)
#define sm_error(...) \
  do {                \
  } while (0)
#endif

typedef Eigen::Matrix<csm_scalar, -1, -1> matrix;
typedef Eigen::Matrix<csm_scalar, -1, 1> vector;
typedef Eigen::Matrix<csm_scalar, 2, 2> matrix2;
typedef Eigen::Matrix<csm_scalar, 2, 1> vector2;
typedef Eigen::Matrix<csm_scalar, 3, 3> matrix3;
typedef Eigen::Matrix<csm_scalar, 3, 1> vector3;

inline matrix zeros(int r, int c) { return matrix::Zero(r, c); }
inline matrix ones(int r, int c) { return matrix::Ones(r, c); }
vector vector_from_array(unsigned int n, const csm_scalar* x);
void vector_to_array(const vector& v, csm_scalar* x);

matrix2 rot(csm_scalar theta);
vector2 vers(csm_scalar theta);
matrix comp_col(const matrix& m1, const matrix& m2);
matrix comp_row(const matrix& m1, const matrix& m2);

vector3 ominus(const vector3& x);

template <int n>
Eigen::Matrix<csm_scalar, n, 1> vector_from_array(const csm_scalar* x) {
  return Eigen::Map<const Eigen::Matrix<csm_scalar, n, 1>>(x);
}

} /* namespace csm */

#endif /* CSMSTRIP_DEPS_H_ */
