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

#include <complex>
#include "scalar.h"

namespace csm {

namespace {

constexpr csm_scalar k2pow_p1_3 = 1.2599210498948731648;  // 2^(+1/3)
constexpr csm_scalar k2pow_m1_3 = 0.7937005259840997374;  // 2^(-1/3)

} /* namespace */

/*
 * Closed form root computation for degree 4 polynomials of the form
 *  x^4 + a*x^3 + b*x^2 + c*x + d == 0
 */
void quartic_solve(
    csm_scalar a, csm_scalar b, csm_scalar c, csm_scalar d,
    std::complex<csm_scalar> (&solutions)[4]) {

  const csm_scalar aa = a * a;
  const csm_scalar bb = b * b;
  const csm_scalar ab = a * b;

  const csm_scalar t00 = bb - 3 * a * c + 12 * d;
  const csm_scalar t01 =
      27 * d * aa - 72 * b * d + 2 * bb * b - 9 * ab * c + 27 * c * c;
  std::complex<csm_scalar> t02 = -4 * t00 * t00 * t00 + t01 * t01;
  const csm_scalar t03         = -a / 4;

  std::complex<csm_scalar> t04 =
      std::pow(t01 + std::sqrt(t02), csm_scalar(-1. / 3));
  std::complex<csm_scalar> t05 = k2pow_p1_3 * t04 * t00 + k2pow_m1_3 / t04;
  std::complex<csm_scalar> t06 =
      std::sqrt(aa / 4 + (-2 * b + t05) / csm_scalar(3));

  std::complex<csm_scalar> t07 = t03 - t06 / csm_scalar(2);
  std::complex<csm_scalar> t08 = t03 + t06 / csm_scalar(2);
  std::complex<csm_scalar> t09 = aa / 2 - (4 * b + t05) / csm_scalar(3);
  std::complex<csm_scalar> t10 =
      (aa * a - 4 * ab + 8 * c) / (csm_scalar(4) * t06);
  std::complex<csm_scalar> t11 = std::sqrt(t09 + t10) / csm_scalar(2);
  std::complex<csm_scalar> t12 = std::sqrt(t09 - t10) / csm_scalar(2);

  solutions[0] = t07 - t11;
  solutions[1] = t07 + t11;
  solutions[2] = t08 - t12;
  solutions[3] = t08 + t12;
}

} /* namespace csm */
