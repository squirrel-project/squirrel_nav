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

#include "csm.h"

namespace csm {

int compatible(struct sm_params* params, int i, int j) {
  if (!params->do_alpha_test)
    return 1;

  csm_scalar theta0 = 0; /* FIXME */
  if ((params->laser_sens->alpha_valid[i] == 0) ||
      (params->laser_ref->alpha_valid[j] == 0))
    return 1;

  csm_scalar alpha_i   = params->laser_sens->alpha[i];
  csm_scalar alpha_j   = params->laser_ref->alpha[j];
  csm_scalar tolerance = deg2rad(params->do_alpha_test_thresholdDeg);

  /** FIXME remove alpha test */
  csm_scalar theta = angleDiff(alpha_j, alpha_i);
  if (std::abs(angleDiff(theta, theta0)) >
      tolerance + deg2rad(params->max_angular_correction_deg)) {
    return 0;
  } else {
    return 1;
  }
}

void find_correspondences(struct sm_params* params) {
  const LDP laser_ref  = params->laser_ref;
  const LDP laser_sens = params->laser_sens;

  int i;
  for (i = 0; i < laser_sens->nrays; i++) {
    if (!ld_valid_ray(laser_sens, i)) {
      /*			sm_debug("dumb: i %d is invalid \n", i);*/
      ld_set_null_correspondence(laser_sens, i);
      continue;
    }

    csm_scalar* p_i_w = laser_sens->points_w[i].p;

    int j1               = -1;
    csm_scalar best_dist = 10000;

    int from;
    int to;
    int start_cell;
    possible_interval(
        p_i_w, laser_ref, params->max_angular_correction_deg,
        params->max_linear_correction, &from, &to, &start_cell);

    /*		sm_debug("dumb: i %d from  %d to %d \n", i, from, to); */
    int j;
    for (j = from; j <= to; j++) {
      if (!ld_valid_ray(laser_ref, j)) {
        /*				sm_debug("dumb: i %d      j %d invalid\n", i,
         * j);*/
        continue;
      }
      csm_scalar dist = distance_squared_d(p_i_w, laser_ref->points[j].p);
      /*			sm_debug("dumb: i %d j1 %d j %d d %f\n",
       * i,j1,j,dist);*/
      if (dist > square(params->max_correspondence_dist))
        continue;

      if ((-1 == j1) || (dist < best_dist)) {
        if (compatible(params, i, j)) {
          j1        = j;
          best_dist = dist;
        }
      }
    }

    if (j1 == -1) { /* no match */
      ld_set_null_correspondence(laser_sens, i);
      continue;
    }
    /* Do not match with extrema*/
    if (j1 == 0 || (j1 == (laser_ref->nrays - 1))) { /* no match */
      ld_set_null_correspondence(laser_sens, i);
      continue;
    }

    int j2;
    /* No need to compute the second association if we use pre-computed normals,
       we don't really use it in that case, we can put anything arbitrary as j2.
     */
    if (params->use_precoumputed_normals) {
      if (ld_valid_normal(laser_ref, j1)) {
        ld_set_null_correspondence(laser_sens, i);
        continue;
      } else {
        j2 = j1;
      }
    } else {
      int j2up   = ld_next_valid_up(laser_ref, j1);
      int j2down = ld_next_valid_down(laser_ref, j1);
      if ((j2up == -1) && (j2down == -1)) {
        ld_set_null_correspondence(laser_sens, i);
        continue;
      }
      if (j2up == -1) {
        j2 = j2down;
      } else if (j2down == -1) {
        j2 = j2up;
      } else {
        csm_scalar dist_up =
            distance_squared_d(p_i_w, laser_ref->points[j2up].p);
        csm_scalar dist_down =
            distance_squared_d(p_i_w, laser_ref->points[j2down].p);
        j2 = dist_up < dist_down ? j2up : j2down;
      }
    }

    ld_set_correspondence(laser_sens, i, j1, j2);
    laser_sens->corr[i].dist2_j1 = best_dist;
    laser_sens->corr[i].type     = params->use_point_to_line_distance
                                   ? correspondence::corr_pl
                                   : correspondence::corr_pp;
  }
}

} /* namespace csm */
