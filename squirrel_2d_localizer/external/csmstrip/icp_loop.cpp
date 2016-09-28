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
#include <vector>
#include "csm.h"
#include "gpc.h"

namespace csm {

int icp_loop(
    struct sm_params* params, const csm_scalar* q0, csm_scalar* x_new,
    csm_scalar* total_error, int* valid, int* iterations) {
  if (any_nan(q0, 3)) {
    sm_error(
        "icp_loop: Initial pose contains nan: %s\n", friendly_pose(q0).c_str());
    return 0;
  }

  LDP laser_sens = params->laser_sens;
  csm_scalar x_old[3], delta[3], delta_old[3] = {0, 0, 0};
  copy_d(q0, 3, x_old);
  // unsigned int hashes[params->max_iterations];
  std::vector<unsigned int> hashes(params->max_iterations, 0);
  int iteration;

  sm_debug("icp: starting at  q0 =  %s  \n", friendly_pose(x_old).c_str());

  int all_is_okay = 1;

  for (iteration = 0; iteration < params->max_iterations; iteration++) {
    sm_debug("== icp_loop: starting iteration. %d  \n", iteration);

    /** Compute laser_sens's points in laser_ref's coordinates
        by roto-translating by x_old */
    ld_compute_world_coords(laser_sens, x_old);

    /** Find correspondences (the naif or smart way) */
    if (params->use_corr_tricks)
      find_correspondences_tricks(params);
    else
      find_correspondences(params);

    /** If debug_verify_tricks, make sure that find_correspondences_tricks()
        and find_correspondences() return the same answer */
    if (params->debug_verify_tricks)
      debug_correspondences(params);

    /* If not many correspondences, bail out */
    int num_corr         = ld_num_valid_correspondences(laser_sens);
    csm_scalar fail_perc = 0.05;
    if (num_corr < fail_perc * laser_sens->nrays) { /* TODO: arbitrary */
      sm_error("	: before trimming, only %d correspondences.\n", num_corr);
      all_is_okay = 0;
      break;
    }

    /* Kill some correspondences (using dubious algorithm) */
    if (params->outliers_remove_doubles)
      kill_outliers_double(params);

    csm_scalar error = 0;
    /* Trim correspondences */
    kill_outliers_trim(params, &error);
    int num_corr_after = ld_num_valid_correspondences(laser_sens);

    *total_error = error;
    *valid       = num_corr_after;

    sm_debug(
        "  icp_loop: total error: %f  valid %d   mean = %f\n", *total_error,
        *valid, *total_error / *valid);

    /* If not many correspondences, bail out */
    if (num_corr_after < fail_perc * laser_sens->nrays) {
      sm_error(
          "  icp_loop: failed: after trimming, only %d correspondences.\n",
          num_corr_after);
      all_is_okay = 0;
      break;
    }

    /* Compute next estimate based on the correspondences */
    if (!compute_next_estimate(params, x_old, x_new)) {
      sm_error("  icp_loop: Cannot compute next estimate.\n");
      all_is_okay = 0;
      break;
    }

    pose_diff_d(x_new, x_old, delta);

    {
      sm_debug(
          "  icp_loop: killing. laser_sens has %d/%d rays valid,  %d corr "
          "found -> %d after adaptive cut \n",
          count_equal(laser_sens->valid, laser_sens->nrays, 1),
          laser_sens->nrays, num_corr, num_corr_after);
    }
    /** Checks for oscillations */
    hashes[iteration] = ld_corr_hash(laser_sens);

    {
      sm_debug(
          "  icp_loop: it. %d  hash=%d nvalid=%d mean error = %f, x_new= %s\n",
          iteration, hashes[iteration], *valid, *total_error / *valid,
          friendly_pose(x_new).c_str());
    }

    /** PLICP terminates in a finite number of steps! */
    if (params->use_point_to_line_distance) {
      int loop_detected = 0; /* TODO: make function */
      int a;
      for (a = iteration - 1; a >= 0; a--) {
        if (hashes[a] == hashes[iteration]) {
          sm_debug(
              "icpc: oscillation detected (cycle length = %d)\n",
              iteration - a);
          loop_detected = 1;
          break;
        }
      }
      if (loop_detected) {
        break;
      }
    }

    /* This termination criterium is useless when using
       the point-to-line-distance; however, we put it here because
       one can choose to use the point-to-point distance. */
    if (termination_criterion(params, delta)) {
      break;
    }

    copy_d(x_new, 3, x_old);
    copy_d(delta, 3, delta_old);
  }

  *iterations = iteration + 1;

  return all_is_okay;
}

int termination_criterion(struct sm_params* params, const csm_scalar* delta) {
  csm_scalar a = norm_d(delta);
  csm_scalar b = std::abs(delta[2]);
  return (a < params->epsilon_xy) && (b < params->epsilon_theta);
}

int compute_next_estimate(
    struct sm_params* params, const csm_scalar x_old[3], csm_scalar x_new[3]) {
  LDP laser_ref  = params->laser_ref;
  LDP laser_sens = params->laser_sens;

  // struct gpc_corr c[laser_sens->nrays];
  struct gpc_corr dummy;
  std::vector<gpc_corr> c(laser_sens->nrays, dummy);

  int i;
  int k = 0;
  for (i = 0; i < laser_sens->nrays; i++) {
    if (!laser_sens->valid[i])
      continue;

    if (!ld_valid_corr(laser_sens, i))
      continue;

    int j1 = laser_sens->corr[i].j1;
    int j2 = laser_sens->corr[i].j2;

    c[k].valid = 1;

    if (laser_sens->corr[i].type == correspondence::corr_pl) {

      c[k].p[0] = laser_sens->points[i].p[0];
      c[k].p[1] = laser_sens->points[i].p[1];
      c[k].q[0] = laser_ref->points[j1].p[0];
      c[k].q[1] = laser_ref->points[j1].p[1];

      csm_scalar cos_alpha;
      csm_scalar sin_alpha;
      if (params->use_precoumputed_normals) {
        cos_alpha = laser_ref->normals[j1].n[0];
        sin_alpha = laser_ref->normals[j1].n[1];
      } else {
        /** TODO: here we could use the estimated alpha */
        csm_scalar diff[2];
        diff[0] = laser_ref->points[j1].p[0] - laser_ref->points[j2].p[0];
        diff[1] = laser_ref->points[j1].p[1] - laser_ref->points[j2].p[1];
        csm_scalar one_on_norm =
            1 /
            std::hypot(
                diff[0], diff[1]);  // sqrt(diff[0]*diff[0]+diff[1]*diff[1]);
        csm_scalar normal[2];
        normal[0] = +diff[1] * one_on_norm;
        normal[1] = -diff[0] * one_on_norm;

        cos_alpha = normal[0];
        sin_alpha = normal[1];
      }

      c[k].C[0][0] = cos_alpha * cos_alpha;
      c[k].C[1][0] = c[k].C[0][1] = cos_alpha * sin_alpha;
      c[k].C[1][1]                = sin_alpha * sin_alpha;

/*			sm_debug("k=%d, i=%d sens_phi: %fdeg, j1=%d j2=%d,
   alpha_seg=%f, cos=%f sin=%f \n", k,i,
                                rad2deg(laser_sens->theta[i]), j1,j2,
   atan2(sin_alpha,cos_alpha), cos_alpha,sin_alpha);*/

#if 0
			/* Note: it seems that because of numerical errors this matrix might be
			   not semidef positive. */
			csm_scalar det = c[k].C[0][0] * c[k].C[1][1] - c[k].C[0][1] * c[k].C[1][0];
			csm_scalar trace = c[k].C[0][0] + c[k].C[1][1];
			
			int semidef = (det >= 0) && (trace>0);
			if(!semidef) {
	/*			printf("%d: Adjusting correspondence weights\n",i);*/
				csm_scalar eps = -det;
				c[k].C[0][0] += 2*sqrt(eps);
				c[k].C[1][1] += 2*sqrt(eps);
			}
#endif
    } else {
      c[k].p[0] = laser_sens->points[i].p[0];
      c[k].p[1] = laser_sens->points[i].p[1];

      projection_on_segment_d(
          laser_ref->points[j1].p, laser_ref->points[j2].p,
          laser_sens->points_w[i].p, c[k].q);

      /* Identity matrix */
      c[k].C[0][0] = 1;
      c[k].C[1][0] = 0;
      c[k].C[0][1] = 0;
      c[k].C[1][1] = 1;
    }

    csm_scalar factor = 1;

    /* Scale the correspondence weight by a factor concerning the
       information in this reading. */
    if (params->use_ml_weights) {
      int have_alpha   = 0;
      csm_scalar alpha = 0;
      if (!is_nan(laser_ref->true_alpha[j1])) {
        alpha      = laser_ref->true_alpha[j1];
        have_alpha = 1;
      } else if (laser_ref->alpha_valid[j1]) {
        alpha = laser_ref->alpha[j1];
        ;
        have_alpha = 1;
      } else
        have_alpha = 0;

      if (have_alpha) {
        csm_scalar pose_theta = x_old[2];
        /** Incidence of the ray
                Note that alpha is relative to the first scan (not the world)
                and that pose_theta is the angle of the second scan with
                respect to the first, hence it's ok. */
        csm_scalar beta = alpha - (pose_theta + laser_sens->theta[i]);
        factor          = 1 / square(std::cos(beta));
      } else {
        static int warned_before = 0;
        if (!warned_before) {
          sm_error(
              "Param use_ml_weights was active, but not valid alpha[] or "
              "true_alpha[]."
              "Perhaps, if this is a single ray not having alpha, you should "
              "mark it as inactive.\n");
          sm_error("Writing laser_ref: \n");
          /*ld_write_as_json(laser_ref, stderr);*/
          warned_before = 1;
        }
      }
    }

    /* Weight the points by the sigma in laser_sens */
    if (params->use_sigma_weights) {
      if (!is_nan(laser_sens->readings_sigma[i])) {
        factor *= 1 / square(laser_sens->readings_sigma[i]);
      } else {
        static int warned_before = 0;
        if (!warned_before) {
          sm_error(
              "Param use_sigma_weights was active, but the field "
              "readings_sigma[] was not filled in.\n");
          sm_error("Writing laser_sens: \n");
          /*ld_write_as_json(laser_sens, stderr);*/
        }
      }
    }

    c[k].C[0][0] *= factor;
    c[k].C[1][0] *= factor;
    c[k].C[0][1] *= factor;
    c[k].C[1][1] *= factor;

    k++;
  }

  int ok = gpc_solve(k, c, x_new);
  if (!ok) {
    sm_error("gpc_solve_valid failed\n");
    return 0;
  }

  csm_scalar old_error = gpc_total_error(c, k, x_old);
  csm_scalar new_error = gpc_total_error(c, k, x_new);

  sm_debug(
      "\tcompute_next_estimate: old error: %f  x_old= %s \n", old_error,
      friendly_pose(x_old).c_str());
  sm_debug(
      "\tcompute_next_estimate: new error: %f  x_new= %s \n", new_error,
      friendly_pose(x_new).c_str());
  sm_debug(
      "\tcompute_next_estimate: new error - old_error: %g \n",
      new_error - old_error);

  csm_scalar epsilon = 0.000001;
  if (new_error > old_error + epsilon) {
    sm_error(
        "\tcompute_next_estimate: something's fishy here! Old error: %lf  new "
        "error: %lf  x_old %lf %lf %lf x_new %lf %lf %lf\n",
        old_error, new_error, x_old[0], x_old[1], x_old[2], x_new[0], x_new[1],
        x_new[2]);
  }

  return 1;
}

} /* namespace csm */
