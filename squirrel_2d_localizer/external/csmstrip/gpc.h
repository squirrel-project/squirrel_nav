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
/*
// GPC: A library for the solution of General Point Correspondence problems.
// Copyright (C) 2006 Andrea Censi (andrea at censi dot org)
*/

#ifndef H_GENERAL_POINT_CORRESPONDENCE
#define H_GENERAL_POINT_CORRESPONDENCE

#include <vector>
#include "scalar.h"

namespace csm {

struct gpc_corr {
  csm_scalar p[2];
  csm_scalar q[2];

  csm_scalar C[2][2];

  int valid;
};

/**
// This program solves the general point correspondences problem:
// to find a translation $t$ and rotation $\theta$ that minimize
//
//  \sum_k (rot(theta)*c[k].p+t-c[k].q)' * c[k].C * (rot(theta)*c[k].p+t-c[k].q)
//
// (see the attached documentation for details).
*/

#define TRACE_ALGO 0

/* Set to 1 to force a check that C is positive semidef.
   Note that you will have some numerical errors! */
#define GPC_CHECK_SEMIDEF 0

/** if c[k].valid is 0, the correspondence is not used */
int gpc_solve(int K, const std::vector<gpc_corr>&, csm_scalar* x);

/* Some utilities functions */

/** Computes error for a single correspondence */
csm_scalar gpc_error(const struct gpc_corr* co, const csm_scalar* x);

csm_scalar gpc_total_error(
    const std::vector<gpc_corr>& co, int n, const csm_scalar* x);

} /* namespace csm */

#endif
