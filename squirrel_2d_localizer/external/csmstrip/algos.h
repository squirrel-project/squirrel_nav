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
#ifndef H_SCAN_MATCHING_LIB
#define H_SCAN_MATCHING_LIB

#include "csm_deps.h"
#include "laser_data.h"

namespace csm {

struct sm_params {
  /** First scan ("ref"erence scan) */
  LDP laser_ref;
  /** Second scan ("sens"or scan) */
  LDP laser_sens;

  /** Where to start */
  csm_scalar first_guess[3];

  /** Maximum angular displacement between scans (deg)*/
  csm_scalar max_angular_correction_deg;
  /** Maximum translation between scans (m) */
  csm_scalar max_linear_correction;

  /** When to stop */
  int max_iterations;
  /** A threshold for stopping. */
  csm_scalar epsilon_xy;
  /** A threshold for stopping. */
  csm_scalar epsilon_theta;

  /** Maximum distance for a correspondence to be valid */
  csm_scalar max_correspondence_dist;
  /** Use smart tricks for finding correspondences. Only influences speed; not
   * convergence. */
  int use_corr_tricks;

  /** Use pre-computed normals for point-to-line distance */
  int use_precoumputed_normals;

  /** Restart if error under threshold (0 or 1)*/
  int restart;
  /** Threshold for restarting */
  csm_scalar restart_threshold_mean_error;
  /** Displacement for restarting */
  csm_scalar restart_dt;
  /** Displacement for restarting */
  csm_scalar restart_dtheta;

  /* Functions concerning discarding correspondences.
     THESE ARE MAGIC NUMBERS -- and they need to be tuned. */

  /** Percentage of correspondences to consider: if 0.9,
      always discard the top 10% of correspondences with more error */
  csm_scalar outliers_maxPerc;

  /** Parameters describing a simple adaptive algorithm for discarding.
      1) Order the errors.
          2) Choose the percentile according to outliers_adaptive_order.
             (if it is 0.7, get the 70% percentile)
          3) Define an adaptive threshold multiplying outliers_adaptive_mult
             with the value of the error at the chosen percentile.
          4) Discard correspondences over the threshold.

          This is useful to be conservative; yet remove the biggest errors.
  */
  csm_scalar outliers_adaptive_order; /* 0.7 */
  csm_scalar outliers_adaptive_mult;  /* 2 */

  /** Do not allow two different correspondences to share a point */
  int outliers_remove_doubles;

  /* Functions that compute and use point orientation for defining matches. */
  /** For now, a very simple max-distance clustering algorithm is used */
  csm_scalar clustering_threshold;
  /** Number of neighbour rays used to estimate the orientation.*/
  int orientation_neighbourhood;
  /** Discard correspondences based on the angles */
  int do_alpha_test;
  csm_scalar do_alpha_test_thresholdDeg;

  /** I believe this trick is documented in one of the papers by Guttman (but I
     can't find
      the reference). Or perhaps I was told by him directly.

          If you already have a guess of the solution, you can compute the polar
     angle
          of the points of one scan in the new position. If the polar angle is
     not a monotone
          function of the readings index, it means that the surface is not
     visible in the
          next position. If it is not visible, then we don't use it for
     matching.

          This is confusing without a picture! To understand what's going on,
     make a drawing
          in which a surface is not visible in one of the poses.

          Implemented in the function visibilityTest().
  */
  int do_visibility_test;

  /** If 1, use PlICP; if 0, use vanilla ICP. */
  int use_point_to_line_distance;

  /** If 1, the field "true_alpha" is used to compute the incidence
      beta, and the factor (1/cos^2(beta)) used to weight the impact
      of each correspondence. This works fabolously if doing localization,
      that is the first scan has no noise.
          If "true_alpha" is not available, it uses "alpha".
  */
  int use_ml_weights;

  /* If 1, the field "readings_sigma" is used to weight the correspondence by
   * 1/sigma^2 */
  int use_sigma_weights;

  /** Use the method in http://purl.org/censi/2006/icpcov to compute
      the matching covariance. */
  int do_compute_covariance;

  /** Checks that find_correspondences_tricks give the right answer */
  int debug_verify_tricks;

  /** Pose of sensor with respect to robot: used for computing
      the first estimate given the odometry. */
  csm_scalar laser[3];

  /** Noise in the scan */
  csm_scalar sigma;

  /** mark as invalid ( = don't use ) rays outside of this interval */
  csm_scalar min_reading, max_reading;

  /** max angle for laser scan **/
  csm_scalar max_theta;

  /** min angle for laser scan **/
  csm_scalar min_theta;

  /** min association to be valid **/
  csm_scalar min_associations;
};

struct sm_result {
  /** 1 if the result is valid */
  int valid;

  /** Scan matching result (x,y,theta) */
  csm_scalar x[3];

  /** Number of iterations done */
  int iterations;
  /** Number of valid correspondence in the end */
  int nvalid;
  /** Total correspondence error */
  csm_scalar error;

/** Fields used for covariance computation */
#ifndef RUBY
  matrix cov_x_m;
  matrix dx_dy1_m;
  matrix dx_dy2_m;
#endif
};

void sm_icp(struct sm_params* input, struct sm_result* output);

} /* namespace csm */

#endif
