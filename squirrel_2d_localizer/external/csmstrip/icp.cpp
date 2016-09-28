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
#include <cmath>
#include "csm.h"

namespace csm {

void ld_invalid_if_outside(
    LDP ld, csm_scalar min_reading, csm_scalar max_reading) {
  int i;
  for (i = 0; i < ld->nrays; i++) {
    if (!ld_valid_ray(ld, i))
      continue;
    csm_scalar r = ld->readings[i];
    if (r <= min_reading || r > max_reading)
      ld->valid[i] = 0;
  }
}

void sm_icp(struct sm_params* params, struct sm_result* res) {
  res->valid = 0;

  LDP laser_ref  = params->laser_ref;
  LDP laser_sens = params->laser_sens;

  if (!ld_valid_fields(laser_ref) || !ld_valid_fields(laser_sens)) {
    return;
  }

  sm_debug(
      "sm_icp: laser_sens has %d/%d; laser_ref has %d/%d rays valid\n",
      count_equal(laser_sens->valid, laser_sens->nrays, 1), laser_sens->nrays,
      count_equal(laser_ref->valid, laser_ref->nrays, 1), laser_ref->nrays);

  /** Mark as invalid the rays outside of (min_reading, max_reading] */
  ld_invalid_if_outside(laser_ref, params->min_reading, params->max_reading);
  ld_invalid_if_outside(laser_sens, params->min_reading, params->max_reading);

  sm_debug(
      "sm_icp:  laser_sens has %d/%d; laser_ref has %d/%d rays valid (after "
      "removing outside interval [%f, %f])\n",
      count_equal(laser_sens->valid, laser_sens->nrays, 1), laser_sens->nrays,
      count_equal(laser_ref->valid, laser_ref->nrays, 1), laser_ref->nrays,
      params->min_reading, params->max_reading);

  if (params->use_corr_tricks || params->debug_verify_tricks)
    ld_create_jump_tables(laser_ref);

  ld_compute_cartesian(laser_ref);
  ld_compute_cartesian(laser_sens);

  if (params->do_alpha_test) {
    ld_simple_clustering(laser_ref, params->clustering_threshold);
    ld_compute_orientation(
        laser_ref, params->orientation_neighbourhood, params->sigma);
    ld_simple_clustering(laser_sens, params->clustering_threshold);
    ld_compute_orientation(
        laser_sens, params->orientation_neighbourhood, params->sigma);
  }

  vector x_new(3);
  vector x_old = vector_from_array(3, params->first_guess);

  if (params->do_visibility_test) {
    sm_debug("laser_ref:\n");
    visibilityTest(laser_ref, x_old);

    sm_debug("laser_sens:\n");
    vector minus_x_old = ominus(x_old);
    visibilityTest(laser_sens, minus_x_old);
  }

  csm_scalar error;
  int iterations;
  int nvalid;
  if (!icp_loop(
          params, x_old.data(), x_new.data(), &error, &nvalid, &iterations)) {
    sm_error("icp: ICP failed for some reason. \n");
    res->valid      = 0;
    res->iterations = iterations;
    res->nvalid     = 0;
  } else {
    /* It was succesfull */

    csm_scalar best_error = error;
    vector best_x         = x_new;

    if (params->restart &&
        (error / nvalid) > (params->restart_threshold_mean_error)) {
      sm_debug(
          "Restarting: %f > %f \n", (error / nvalid),
          (params->restart_threshold_mean_error));
      csm_scalar dt  = params->restart_dt;
      csm_scalar dth = params->restart_dtheta;
      sm_debug("icp_loop: dt = %f dtheta= %f deg\n", dt, rad2deg(dth));

      csm_scalar perturb[6][3] = {{dt, 0, 0},  {-dt, 0, 0}, {0, dt, 0},
                                  {0, -dt, 0}, {0, 0, dth}, {0, 0, -dth}};

      int a;
      for (a = 0; a < 6; a++) {
        sm_debug("-- Restarting with perturbation #%d\n", a);
        struct sm_params my_params = *params;
        vector start(3);
        start << x_new[0] + perturb[a][0], x_new[1] + perturb[a][1],
            x_new[2] + perturb[a][2];
        vector x_a(3);

        csm_scalar my_error;
        int my_valid;
        int my_iterations;
        if (!icp_loop(
                &my_params, start.data(), x_a.data(), &my_error, &my_valid,
                &my_iterations)) {
          sm_error("Error during restart #%d/%d. \n", a, 6);
          break;
        }
        iterations += my_iterations;

        if (my_error < best_error) {
          sm_debug(
              "--Perturbation #%d resulted in error %f < %f\n", a, my_error,
              best_error);
          best_x     = x_a;
          best_error = my_error;
        }
      }
    }

    /* At last, we did it. */
    res->valid = 1;
    vector_to_array(best_x, res->x);
    sm_debug("icp: final x =  %s  \n", friendly_pose(best_x.data()).c_str());

    if (params->do_compute_covariance) {

      matrix cov0_x, dx_dy1, dx_dy2;
      compute_covariance_exact(
          laser_ref, laser_sens, best_x, cov0_x, dx_dy1, dx_dy2);

      matrix cov_x = square(params->sigma) * cov0_x;

      res->cov_x_m  = cov_x;
      res->dx_dy1_m = dx_dy1;
      res->dx_dy2_m = dx_dy2;
    }

    res->error      = best_error;
    res->iterations = iterations;
    res->nvalid     = nvalid;
  }
}

} /* namespace csm */
