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

#ifndef H_LASER_DATA_INLINE
#define H_LASER_DATA_INLINE

#include <cmath>
#include <limits>
#include "scalar.h"

namespace csm {

/* Simple inline functions */

/*#warning Seen ld_valid_ray*/

INLINE int ld_valid_ray(LDP ld, int i) {
  return (i >= 0) && (i < ld->nrays) && (ld->valid[i]);
}

INLINE int ld_valid_alpha(LDP ld, int i) { return ld->alpha_valid[i] != 0; }

INLINE int ld_valid_normal(LDP ld, int i) {
  return std::isfinite(ld->normals[i].n[0]) &&
         std::isfinite(ld->normals[i].n[1]);
}

INLINE void ld_set_null_correspondence(LDP ld, int i) {
  ld->corr[i].valid = 0;
  ld->corr[i].j1    = -1;
  ld->corr[i].j2    = -1;
  ld->corr[i].dist2_j1 =
      std::numeric_limits<csm_scalar>::quiet_NaN();  // GSL_NAN;
}

INLINE void ld_set_correspondence(LDP ld, int i, int j1, int j2) {
  ld->corr[i].valid = 1;
  ld->corr[i].j1    = j1;
  ld->corr[i].j2    = j2;
}

/** -1 if not found */

INLINE int ld_next_valid(LDP ld, int i, int dir) {
  int j;
  for (j = i + dir; (j < ld->nrays) && (j >= 0) && !ld_valid_ray(ld, j);
       j += dir)
    ;
  return ld_valid_ray(ld, j) ? j : -1;
}

INLINE int ld_next_valid_up(LDP ld, int i) { return ld_next_valid(ld, i, +1); }

INLINE int ld_next_valid_down(LDP ld, int i) {
  return ld_next_valid(ld, i, -1);
}

INLINE int ld_valid_corr(LDP ld, int i) { return ld->corr[i].valid; }

} /*namespace csm */

#endif
