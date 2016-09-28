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

#ifndef _H_ICP_
#define _H_ICP_

//#include <gpc/gpc.h>

#include "csm_deps.h"
#include "laser_data.h"

namespace csm {

/** This sets the stage.  */
void sm_icp(struct sm_params* params, struct sm_result* res);

/** This is the meat */
int icp_loop(
    struct sm_params* params, const csm_scalar* q0, csm_scalar* x_new,
    csm_scalar* total_error, int* nvalid, int* iterations);

/** This is the beef: computing in closed form the next estimate
    given the correspondences. */
int compute_next_estimate(
    struct sm_params* params, const csm_scalar x_old[3], csm_scalar x_new[3]);

/** This termination criterium use epsilon_xy and epsilon_th.
   It is useless when using
   the point-to-line-distance; however, we put it here because
   one can choose to use the point-to-point distance. */
int termination_criterion(struct sm_params* params, const csm_scalar* delta);

/** Naif algorithm */
void find_correspondences(struct sm_params* params);
/** Smart algorithm */
void find_correspondences_tricks(struct sm_params* params);
/** Checks that find_correspondences_tricks and find_correspondences behave the
   same.
        Exit(-1) on error. */
void debug_correspondences(struct sm_params* params);

void kill_outliers_trim(struct sm_params* params, csm_scalar* total_error);
void kill_outliers_double(struct sm_params* params);

void compute_covariance_exact(
    LDP laser_ref, LDP laser_sens, const vector& x, matrix& cov0_x,
    matrix& dx_dy1, matrix& dx_dy2);

void visibilityTest(LDP ld, const vector& x_old);

/** Marks a ray invalid if reading is outside range [min_reading, max_reading].
 */
void ld_invalid_if_outside(
    LDP ld, csm_scalar min_reading, csm_scalar max_reading);

void swap_double(csm_scalar* a, csm_scalar* b);

} /* namespace csm */

#endif
