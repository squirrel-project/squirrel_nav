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

#include <cmath>
#include "laser_data.h"

namespace csm {

/*
   A very very simple clustering algorithm.
   Try threshold = 5*sigma
*/

void ld_simple_clustering(LDP ld, csm_scalar threshold) {
  int cluster             = -1;
  csm_scalar last_reading = 0; /* I have to initialize it
    explicitely or else gcc complains it might be uninitialized.
    Stupid compiler, it cannot even solve the halting problem. */

  int i;
  for (i = 0; i < ld->nrays; i++) {
    /* Skip if not valid */
    if (!ld_valid_ray(ld, i)) {
      ld->cluster[i] = -1;
      continue;
    }
    /* If this is the first valid point, assign cluster #0 */
    if (-1 == cluster)
      cluster = 0;
    else
        /* Else, start a new cluster if distance is above threshold */
        if (std::abs(last_reading - ld->readings[i]) > threshold)
      cluster++;

    ld->cluster[i] = cluster;
    last_reading   = ld->readings[i];
  }

  /* TODO: set to -1 the one-point clusters */
}

} /* namespace csm */
