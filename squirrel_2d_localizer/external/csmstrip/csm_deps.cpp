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

#include "csm_deps.h"
#include <stdarg.h>
#include <cmath>
#include <string>

namespace csm {

#ifdef CSMSTRIP_DEBUG

void sm_debug(const char* msg, ...) {
  static bool prevlf = true;
  std::string s;
  if (prevlf) {
    s = std::string("[DEBUG] ") + msg;
  } else {
    s = msg;
  }
  va_list args;
  va_start(args, msg);
  vfprintf(stdout, s.c_str(), args);
  va_end(args);
  prevlf = (s[s.size() - 1] == '\n');
}

void sm_error(const char* msg, ...) {
  static bool prevlf = true;
  std::string s;
  if (prevlf) {
    s = std::string("[ERROR] ") + msg;
  } else {
    s = msg;
  }
  va_list args;
  va_start(args, msg);
  vfprintf(stderr, s.c_str(), args);
  va_end(args);
  prevlf = (s[s.size() - 1] == '\n');
}

void sm_info(const char* msg, ...) {
  static bool prevlf = true;
  std::string s;
  if (prevlf) {
    s = std::string("[INFO ] ") + msg;
  } else {
    s = msg;
  }
  va_list args;
  va_start(args, msg);
  vfprintf(stdout, s.c_str(), args);
  va_end(args);
  prevlf = (s[s.size() - 1] == '\n');
}
#endif

vector vector_from_array(unsigned int n, const csm_scalar* x) {
  return Eigen::Map<const vector>(x, n);
}

void vector_to_array(const vector& v, csm_scalar* x) {
  for (int i = 0; i < v.size(); i++)
    x[i]     = v[i];
}

matrix2 rot(csm_scalar theta) {
  const csm_scalar c = std::cos(theta), s = std::sin(theta);
  matrix2 R;
  R << c, -s, s, c;
  return R;
}

vector2 vers(csm_scalar theta) {
  vector2 v;
  v << std::cos(theta), std::sin(theta);
  return v;
}

matrix comp_col(const matrix& m1, const matrix& m2) {
  matrix v(m1.rows() + m2.rows(), m1.cols());
  v << m1, m2;
  return v;
}

matrix comp_row(const matrix& m1, const matrix& m2) {
  matrix v(m1.rows(), m1.cols() + m2.cols());
  v << m1, m2;
  return v;
}

vector3 ominus(const vector3& x) {
  csm_scalar c = std::cos(x[2]);
  csm_scalar s = std::sin(x[2]);
  vector3 res;
  res << -c * x[0] - s * x[1], s * x[0] - c * x[1], -x[2];
  return res;
}

} /* namespace csm */
