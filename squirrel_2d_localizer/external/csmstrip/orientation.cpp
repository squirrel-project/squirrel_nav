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
#include <vector>

#include "csm.h"

namespace csm {

void find_neighbours(
    LDP ld, int i, int max_num, std::vector<int>& indexes, size_t* num_found);
void filter_orientation(
    csm_scalar theta0, csm_scalar rho0, size_t n,
    const std::vector<csm_scalar>& thetas, const std::vector<csm_scalar>& rhos,
    csm_scalar* alpha, csm_scalar* cov0_alpha);

/** Requires the "cluster" field to be set */
void ld_compute_orientation(LDP ld, int size_neighbourhood, csm_scalar sigma) {
  int i;
  for (i = 0; i < ld->nrays; i++) {
    if (!ld_valid_ray(ld, i) || (ld->cluster[i] == -1)) {
      ld->alpha[i] = std::numeric_limits<csm_scalar>::quiet_NaN();  // GSL_NAN;
      ld->cov_alpha[i] =
          std::numeric_limits<csm_scalar>::quiet_NaN();  // GSL_NAN;
      ld->alpha_valid[i] = 0;
      continue;
    }

    // int neighbours[size_neighbourhood*2];
    std::vector<int> neighbours(size_neighbourhood * 2, 0);
    size_t num_neighbours;
    find_neighbours(ld, i, size_neighbourhood, neighbours, &num_neighbours);

    if (0 == num_neighbours) {
      ld->alpha[i] = std::numeric_limits<csm_scalar>::quiet_NaN();  // GSL_NAN;
      ld->cov_alpha[i] =
          std::numeric_limits<csm_scalar>::quiet_NaN();  // GSL_NAN;
      ld->alpha_valid[i] = 0;
      continue;
    }

    /*		printf("orientation for i=%d:\n",i); */
    // csm_scalar thetas[num_neighbours];
    std::vector<csm_scalar> thetas(num_neighbours, 0.0);
    // csm_scalar readings[num_neighbours];
    std::vector<csm_scalar> readings(num_neighbours, 0.0);
    size_t a = 0;
    for (a = 0; a < num_neighbours; a++) {
      thetas[a]   = ld->theta[neighbours[a]];
      readings[a] = ld->readings[neighbours[a]];
      /* printf(" j = %d theta = %f rho = %f\n", neighbours[a],
       * thetas[a],readings[a]); */
    }

    csm_scalar alpha = 42, cov0_alpha = 32;
    filter_orientation(
        ld->theta[i], ld->readings[i], num_neighbours, thetas, readings, &alpha,
        &cov0_alpha);

    if (is_nan(alpha)) {
      ld->alpha[i]       = std::numeric_limits<csm_scalar>::quiet_NaN();
      ld->cov_alpha[i]   = std::numeric_limits<csm_scalar>::quiet_NaN();
      ld->alpha_valid[i] = 0;
    } else {
      ld->alpha[i]       = alpha;
      ld->cov_alpha[i]   = cov0_alpha * square(sigma);
      ld->alpha_valid[i] = 1;
    }
    /* printf("---------- i = %d alpha = %f sigma=%f cov_alpha = %f\n", i,
     * alpha, ld->cov_alpha[i]);*/
  }
}

/** A very cool algorithm for finding the orientation */

void filter_orientation(
    csm_scalar theta0, csm_scalar rho0, size_t n,
    const std::vector<csm_scalar>& thetas, const std::vector<csm_scalar>& rhos,
    csm_scalar* alpha, csm_scalar* cov0_alpha) {

  /* Y = L x + R epsilon */
  matrix Y = zeros(n, 1);
  matrix L = ones(n, 1);
  matrix R = zeros(n, n + 1);

  size_t i;
  for (i = 0; i < n; i++) {
    Y(i, 0)     = (rhos[i] - rho0) / (thetas[i] - theta0);
    R(i, 0)     = -1 / (thetas[i] - theta0);
    R(i, i + 1) = +1 / (thetas[i] - theta0);
  }

  matrix eRinv   = (R * R.transpose()).eval().inverse();
  matrix vcov_f1 = (L.transpose() * eRinv * L).eval().inverse();
  matrix vf1     = vcov_f1 * L.transpose() * eRinv * Y;

  csm_scalar cov_f1 = vcov_f1(0, 0);
  csm_scalar f1     = vf1(0, 0);

  *alpha = theta0 - atan(f1 / rho0);

  if (std::cos(*alpha) * std::cos(theta0) +
          std::sin(*alpha) * std::sin(theta0) >
      0)
    *alpha = *alpha + M_PI;

  csm_scalar dalpha_df1  = rho0 / (square(rho0) + square(f1));
  csm_scalar dalpha_drho = -f1 / (square(rho0) + square(f1));

  *cov0_alpha = square(dalpha_df1) * cov_f1 + square(dalpha_drho);

  //#ifndef WINDOWS
  //        if(std::isnan(*alpha)) {
  //#else
  //        if(_isnan(*alpha)) {
  //#endif
  //		egsl_print("Y",Y);
  //		egsl_print("L",L);
  //		egsl_print("R",R);
  //		egsl_print("eRinv",eRinv);
  //		egsl_print("vcov_f1",vcov_f1);
  //
  //		printf("   f1 = %f cov =%f \n", f1,cov_f1);
  //		printf("   f1/rho = %f \n", f1/rho0);
  //		printf("   atan = %f \n", atan(f1/rho0));
  //		printf("   theta0= %f \n", theta0);
  //	}
}

/* indexes: an array of size "max_num*2" */
void find_neighbours(
    LDP ld, int i, int max_num, std::vector<int>& indexes, size_t* num_found) {
  *num_found = 0;
  int up = i;
  while ((up + 1 <= i + max_num) && (up + 1 < ld->nrays) &&
         ld_valid_ray(ld, up + 1) && (ld->cluster[up + 1] == ld->cluster[i])) {
    up += 1;
    indexes[(*num_found)++] = up;
  }
  int down = i;
  while ((down >= i - max_num) && (down - 1 >= 0) &&
         ld_valid_ray(ld, down - 1) &&
         (ld->cluster[down - 1] == ld->cluster[i])) {
    down -= 1;
    indexes[(*num_found)++] = down;
  }
}

} /* namespace csm */
