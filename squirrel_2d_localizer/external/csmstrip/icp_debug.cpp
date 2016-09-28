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

#include <string.h>
#include <vector>
#include "csm.h"

namespace csm {

void debug_correspondences(struct sm_params* params) {
  LDP laser_sens = params->laser_sens;
  /** Do the test */
  find_correspondences_tricks(params);
  // struct correspondence c1[laser_sens->nrays];
  std::vector<correspondence> c1;
  c1.reserve(laser_sens->nrays);

  struct correspondence* c2 = laser_sens->corr;
  // memcpy(c1, c2, sizeof(struct correspondence) * laser_sens->nrays);
  c1.assign(c2, c2 + laser_sens->nrays);
  long hash1 = ld_corr_hash(laser_sens);
  find_correspondences(params);
  long hash2 = ld_corr_hash(laser_sens);
  if (hash1 != hash2) {
    sm_error("find_correspondences_tricks might be buggy\n");
    int i = 0;
    for (i = 0; i < laser_sens->nrays; i++) {
      if ((c1[i].valid != c2[i].valid) || (c1[i].j1 != c2[i].j1) ||
          (c1[i].j2 != c2[i].j2)) {
        sm_error(
            "\t   tricks: c1[%d].valid = %d j1 = %d  j2 = %d  dist2_j1 = %f\n",
            i, c1[i].valid, c1[i].j1, c1[i].j2, c1[i].dist2_j1);
        sm_error(
            "\tno tricks: c2[%d].valid = %d j1 = %d  j2 = %d  dist2_j1 = %f\n",
            i, c2[i].valid, c2[i].j1, c2[i].j2, c2[i].dist2_j1);
      }
    }
    if (1)
      exit(-1);
  }
}

} /* namespace csm */
