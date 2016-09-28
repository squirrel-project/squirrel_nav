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

/** expects cartesian valid */
void visibilityTest(LDP laser_ref, const vector& u) {

  // csm_scalar theta_from_u[laser_ref->nrays];
  std::vector<csm_scalar> theta_from_u(laser_ref->nrays, 0.0);

  int j;
  for (j = 0; j < laser_ref->nrays; j++) {
    if (!ld_valid_ray(laser_ref, j))
      continue;
    theta_from_u[j] = atan2(
        u[1] - laser_ref->points[j].p[1], u[0] - laser_ref->points[j].p[0]);
  }

  sm_debug("\tvisibility: Found outliers: ");
  int invalid = 0;
  for (j = 1; j < laser_ref->nrays; j++) {
    if (!ld_valid_ray(laser_ref, j) || !ld_valid_ray(laser_ref, j - 1))
      continue;
    if (theta_from_u[j] < theta_from_u[j - 1]) {
      laser_ref->valid[j] = 0;
      invalid++;
      sm_debug("%d ", j);
    }
  }
  sm_debug("\n");
}

/**
        If multiple points in laser_sens match to the same point in laser_ref,
        only the nearest one wins.

        Uses: laser_sens->corr, laser_sens->p
        Modifies: laser_sens->corr
 */
void kill_outliers_double(struct sm_params* params) {
  csm_scalar threshold = 3; /* TODO: add as configurable */

  LDP laser_ref  = params->laser_ref;
  LDP laser_sens = params->laser_sens;

  // csm_scalar dist2_i[laser_sens->nrays];
  std::vector<csm_scalar> dist2_i(laser_sens->nrays, 0.0);
  // csm_scalar dist2_j[laser_ref->nrays];
  std::vector<csm_scalar> dist2_j(laser_ref->nrays, 0.0);
  int j;
  for (j       = 0; j < laser_ref->nrays; j++)
    dist2_j[j] = 1000000;

  int i;
  for (i = 0; i < laser_sens->nrays; i++) {
    if (!ld_valid_corr(laser_sens, i))
      continue;
    int j1      = laser_sens->corr[i].j1;
    dist2_i[i]  = laser_sens->corr[i].dist2_j1;
    dist2_j[j1] = (std::min)(dist2_j[j1], dist2_i[i]);
  }

  int nkilled = 0;
  for (i = 0; i < laser_sens->nrays; i++) {
    if (!ld_valid_corr(laser_sens, i))
      continue;
    int j1 = laser_sens->corr[i].j1;
    if (dist2_i[i] > (threshold * threshold) * dist2_j[j1]) {
      laser_sens->corr[i].valid = 0;
      nkilled++;
    }
  }
  sm_debug("\tkill_outliers_double: killed %d correspondences\n", nkilled);
}

/**
        Trims the corrispondences using an adaptive algorithm

        Assumes cartesian coordinates computed. (points and points_w)

        So, to disable this:
                outliers_maxPerc = 1
                outliers_adaptive_order = 1

*/
void kill_outliers_trim(struct sm_params* params, csm_scalar* total_error) {

  LDP laser_ref  = params->laser_ref;
  LDP laser_sens = params->laser_sens;

  /* dist2, indexed by k, contains the error for the k-th correspondence */
  int k = 0;
  // csm_scalar dist2[laser_sens->nrays];
  std::vector<csm_scalar> dist2(laser_sens->nrays, 0.0);

  int i;
  // csm_scalar dist[laser_sens->nrays];
  std::vector<csm_scalar> dist(laser_sens->nrays, 0.0);
  /* for each point in laser_sens */
  for (i = 0; i < laser_sens->nrays; i++) {
    /* which has a valid correspondence */
    if (!ld_valid_corr(laser_sens, i)) {
      dist[i] = std::numeric_limits<csm_scalar>::quiet_NaN();
      continue;
    }
    csm_scalar* p_i_w = laser_sens->points_w[i].p;

    int j1 = laser_sens->corr[i].j1;
    int j2 = laser_sens->corr[i].j2;
    /* Compute the distance to the corresponding segment */
    dist[i] = dist_to_segment_d(
        laser_ref->points[j1].p, laser_ref->points[j2].p, p_i_w);
    dist2[k] = dist[i];
    k++;
  }

#if 0	
	csm_scalar dist2_copy[k]; for(i=0;i<k;i++) dist2_copy[i] = dist2[i];
#endif

  /* two errors limits are defined: */
  /* In any case, we don't want more than outliers_maxPerc% */
  int order = (int)floor(k * (params->outliers_maxPerc));
  order     = (std::max)(0, (std::min)(order, k - 1));

  /* The dists for the correspondence are sorted
     in ascending order */
  std::sort(dist2.begin(), dist2.begin() + k);
  csm_scalar error_limit1 = dist2[order];

  /* Then we take a order statics (o*K) */
  /* And we say that the error must be less than alpha*dist(o*K) */
  int order2              = (int)floor(k * params->outliers_adaptive_order);
  order2                  = (std::max)(0, (std::min)(order2, k - 1));
  csm_scalar error_limit2 = params->outliers_adaptive_mult * dist2[order2];

  csm_scalar error_limit = (std::min)(error_limit1, error_limit2);

#if 0
	csm_scalar error_limit1_ho = hoare_selection(dist2_copy, 0, k-1, order);
	csm_scalar error_limit2_ho = error_limit2;
	if((error_limit1_ho != error_limit1) || (error_limit2_ho != error_limit2)) {
		printf("%f == %f    %f  == %f\n",
			error_limit1_ho, error_limit1, error_limit2_ho, error_limit2);
	}
#endif

  sm_debug(
      "\ticp_outliers: maxPerc %f error_limit: fix %f adaptive %f \n",
      params->outliers_maxPerc, error_limit1, error_limit2);

  *total_error = 0;
  int nvalid   = 0;
  for (i = 0; i < laser_sens->nrays; i++) {
    if (!ld_valid_corr(laser_sens, i))
      continue;
    if (dist[i] > error_limit) {
      laser_sens->corr[i].valid = 0;
      laser_sens->corr[i].j1    = -1;
      laser_sens->corr[i].j2    = -1;
    } else {
      nvalid++;
      *total_error += dist[i];
    }
  }

  sm_debug(
      "\ticp_outliers: valid %d/%d (limit: %f) mean error = %f \n", nvalid, k,
      error_limit, *total_error / nvalid);
}

void swap_double(csm_scalar* a, csm_scalar* b) {
  csm_scalar t = *a;
  *a           = *b;
  *b           = t;
}

} /* namespace csm */
