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
#include <iostream>
#include "csm.h"

#define CSM_ORIGINAL_COVARIANCE 1

namespace csm {

// matrix compute_C_k(const vector &p_j1, const vector &p_j2);
// matrix dC_drho(const vector &p1, const vector &p2);

#if CSM_ORIGINAL_COVARIANCE
matrix2 compute_C_k(const vector2& p_j1, const vector2& p_j2) {
  vector2 d = p_j1 - p_j2;

  csm_scalar den = 1 / std::hypot(d[0], d[1]);
  csm_scalar c = -d[1] * den, s = d[0] * den;
  //    csm_scalar alpha = M_PI/2 + atan2(d[1], d[0]);
  //    csm_scalar c = std::cos(alpha); csm_scalar s = std::sin(alpha);
  matrix2 v;
  v << c * c, c * s, c * s, s * s;
  return v;
}

matrix2 dC_drho(const vector2& p1, const vector2& p2) {
  csm_scalar eps = 0.001;

  matrix2 C_k     = compute_C_k(p1, p2);
  vector2 p1b     = p1 + p1.normalized() * eps;
  matrix2 C_k_eps = compute_C_k(p1b, p2);
  return (C_k_eps - C_k) / eps;
}
#endif

void compute_covariance_exact(
    LDP laser_ref, LDP laser_sens, const vector& est, matrix& cov0_x,
    matrix& dx_dy1, matrix& dx_dy2) {
#if CSM_ORIGINAL_COVARIANCE
  matrix d2J_dxdy1 = zeros(3, laser_ref->nrays);
  matrix d2J_dxdy2 = zeros(3, laser_sens->nrays);

  /* the three pieces of d2J_dx2 */
  matrix2 d2J_dt2        = zeros(2, 2);
  vector2 d2J_dt_dtheta  = zeros(2, 1);
  csm_scalar d2J_dtheta2 = 0;

  csm_scalar theta = est.data()[2];
  vector2 t        = vector_from_array<2>(est.data());

  int i;
  for (i = 0; i < laser_sens->nrays; i++) {
    if (!ld_valid_corr(laser_sens, i))
      continue;

    int j1 = laser_sens->corr[i].j1;
    int j2 = laser_sens->corr[i].j2;

    vector2 p_i  = vector_from_array<2>(laser_sens->points[i].p);
    vector2 p_j1 = vector_from_array<2>(laser_ref->points[j1].p);
    vector2 p_j2 = vector_from_array<2>(laser_ref->points[j2].p);

    vector2 v1 = rot(theta + M_PI / 2) * p_i;
    vector2 v2 = rot(theta) * p_i + t - p_j1;
    vector2 v3 = vers(theta + laser_sens->theta[i]);
    vector2 v4 = vers(theta + laser_sens->theta[i] + M_PI / 2);

    matrix2 C_k = compute_C_k(p_j1, p_j2);

    matrix2 d2J_dt2_k       = 2.0 * C_k;
    vector2 d2J_dt_dtheta_k = 2.0 * C_k * v1;

    vector2 v_new = rot(theta + M_PI) * p_i;
    csm_scalar d2J_dtheta2_k =
        2.0 * (v2.transpose() * C_k * v_new + v1.transpose() * C_k * v1).x();
    d2J_dt2 += d2J_dt2_k;
    d2J_dt_dtheta += d2J_dt_dtheta_k;
    d2J_dtheta2 += d2J_dtheta2_k;

    /* for measurement rho_i  in the second scan */
    vector2 d2Jk_dtdrho_i = 2.0 * C_k * v3;
    csm_scalar d2Jk_dtheta_drho_i =
        2.0 * (v2.transpose() * C_k * v4 + v3.transpose() * C_k * v1).x();
    // 		d2J_dxdy2.col(i) += comp_col(d2Jk_dtdrho_i, d2Jk_dtheta_drho_i);
    d2J_dxdy2(0, i) += d2Jk_dtdrho_i[0];
    d2J_dxdy2(1, i) += d2Jk_dtdrho_i[1];
    d2J_dxdy2(2, i) += d2Jk_dtheta_drho_i;

    /* for measurements rho_j1, rho_j2 in the first scan */

    matrix2 dC_drho_j1 = dC_drho(p_j1, p_j2);
    matrix2 dC_drho_j2 = dC_drho(p_j2, p_j1);

    vector2 v_j1 = vers(laser_ref->theta[j1]);

    vector2 d2Jk_dt_drho_j1 = -2.0 * C_k * v_j1 + 2.0 * dC_drho_j1 * v2;
    csm_scalar d2Jk_dtheta_drho_j1 =
        (-2.0 * v_j1.transpose() * C_k * v1 + v2.transpose() * dC_drho_j1 * v1)
            .x();
    //		d2J_dxdy1.col(j1) += comp_col(d2Jk_dt_drho_j1,
    //d2Jk_dtheta_drho_j1);
    d2J_dxdy1(0, j1) += d2Jk_dt_drho_j1[0];
    d2J_dxdy1(1, j1) += d2Jk_dt_drho_j1[1];
    d2J_dxdy1(2, j1) += d2Jk_dtheta_drho_j1;

    /* for measurement rho_j2*/
    vector2 d2Jk_dt_drho_j2        = 2.0 * dC_drho_j2 * v2;
    csm_scalar d2Jk_dtheta_drho_j2 = 2.0 * v2.transpose() * dC_drho_j2 * v1;
    //		d2J_dxdy1.col(j2) += comp_col(d2Jk_dt_drho_j2,
    //d2Jk_dtheta_drho_j2);
    d2J_dxdy1(0, j2) += d2Jk_dt_drho_j2[0];
    d2J_dxdy1(1, j2) += d2Jk_dt_drho_j2[1];
    d2J_dxdy1(2, j2) += d2Jk_dtheta_drho_j2;
  }

  /* composes matrix  d2J_dx2  from the pieces*/
  matrix3 d2J_dx2;
  d2J_dx2 << d2J_dt2, d2J_dt_dtheta, d2J_dt_dtheta.transpose(), d2J_dtheta2;
  //	matrix d2J_dx2   = comp_col( comp_row(    d2J_dt2      , d2J_dt_dtheta),
  //	                             comp_row(d2J_dt_dtheta.transpose(),
  //d2J_dtheta2));

  dx_dy1 = -1.0 * d2J_dx2.inverse().eval() * d2J_dxdy1;
  dx_dy2 = -1.0 * d2J_dx2.inverse().eval() * d2J_dxdy2;

  cov0_x = dx_dy1 * dx_dy1.transpose() + dx_dy2 * dx_dy2.transpose();
#else
  matrix3 d2J_dx2 = matrix3::Zero();
  Eigen::Matrix<csm_scalar, 3, -1> d2J_dxdy1 =
      Eigen::Matrix<csm_scalar, 3, -1>::Zero(3, laser_ref->nrays);
  Eigen::Matrix<csm_scalar, 3, -1> d2J_dxdy2 =
      Eigen::Matrix<csm_scalar, 3, -1>::Zero(3, laser_sens->nrays);
  csm_scalar x = est[0], y = est[1], theta = est[2];

  for (int i = 0; i < laser_sens->nrays; i++) {
    if (!ld_valid_corr(laser_sens, i))
      continue;

    const int j1 = laser_sens->corr[i].j1;
    const int j2 = laser_sens->corr[i].j2;

    const csm_scalar px  = laser_sens->points[i].p[0],
                     py  = laser_sens->points[i].p[1];
    const csm_scalar q1x = laser_ref->points[j1].p[0],
                     q1y = laser_ref->points[j1].p[1];
    const csm_scalar q2x = laser_ref->points[j2].p[0],
                     q2y = laser_ref->points[j2].p[1];
    const csm_scalar pa  = laser_sens->theta[i];
    const csm_scalar q1a = laser_ref->theta[j1], q1r = laser_ref->readings[j1];
    const csm_scalar q2a = laser_ref->theta[j2], q2r = laser_ref->readings[j2];
    const csm_scalar dx = q1x - q2x, dy = q1y - q2y;
    const csm_scalar den = 1 / std::hypot(dx, dy);
    const csm_scalar nx = dy * den, ny = -dx * den;

    const csm_scalar tmp0  = (nx * nx);
    const csm_scalar tmp1  = 2 * nx * ny;
    const csm_scalar tmp2  = -(ny * px);
    const csm_scalar tmp3  = nx * py;
    const csm_scalar tmp4  = std::cos(theta);
    const csm_scalar tmp5  = (tmp2 + tmp3) * tmp4;
    const csm_scalar tmp6  = nx * px;
    const csm_scalar tmp7  = ny * py;
    const csm_scalar tmp8  = tmp6 + tmp7;
    const csm_scalar tmp9  = std::sin(theta);
    const csm_scalar tmp10 = tmp8 * tmp9;
    const csm_scalar tmp11 = tmp10 + tmp5;
    const csm_scalar tmp12 = -2 * nx * tmp11;
    const csm_scalar tmp13 = (ny * ny);
    const csm_scalar tmp14 = ny * px;
    const csm_scalar tmp15 = -(nx * py) + tmp14;
    const csm_scalar tmp16 = tmp15 * tmp4 - tmp8 * tmp9;
    const csm_scalar tmp17 = 2 * ny * tmp16;
    const csm_scalar tmp18 = -(ny * q1y);
    const csm_scalar tmp19 = ny * y;
    const csm_scalar tmp20 = -2 * nx * q1x;
    const csm_scalar tmp21 = 2 * nx * x;
    const csm_scalar tmp22 = -2 * nx * py;
    const csm_scalar tmp23 = -q1y + py * tmp4 + px * tmp9 + y;
    const csm_scalar tmp24 = px * tmp4;
    const csm_scalar tmp25 = -(py * tmp9);
    const csm_scalar tmp26 = -q1x + tmp24 + tmp25 + x;
    const csm_scalar tmp27 = -(py * tmp4);
    const csm_scalar tmp28 = -(px * tmp9);
    const csm_scalar tmp29 = ny * tmp23 + nx * tmp26;
    const csm_scalar tmp30 = nx * tmp4 + ny * tmp9;
    const csm_scalar tmp31 = ny * tmp4 - nx * tmp9;
    const csm_scalar tmp32 = -2 * nx * ny;
    const csm_scalar tmp33 = q1a - q2a;
    const csm_scalar tmp34 =
        (q1r * q1r) + (q2r * q2r) - 2 * q1r * q2r * std::cos(tmp33);
    const csm_scalar tmp35 = std::pow(tmp34, -1.5);
    const csm_scalar tmp36 = std::cos(q1a);
    const csm_scalar tmp37 = std::cos(q2a);
    const csm_scalar tmp38 = std::sin(tmp33);
    const csm_scalar tmp39 = std::sin(q1a);
    const csm_scalar tmp40 = std::sin(q2a);

    d2J_dx2(0, 0) += 2 * tmp0;
    d2J_dx2(0, 1) += tmp1;
    d2J_dx2(0, 2) += tmp12;
    d2J_dx2(1, 1) += 2 * tmp13;
    d2J_dx2(1, 2) += tmp17;
    d2J_dx2(2, 2) += 2 * ((tmp11 * tmp11) +
                          (-(tmp4 * tmp8) - tmp15 * tmp9) *
                              (-(nx * q1x) + tmp18 + tmp19 + tmp4 * tmp8 +
                               tmp15 * tmp9 + nx * x));

    Eigen::Matrix<csm_scalar, 3, 2> d2J_dx_dn;
    d2J_dx_dn << 2 * (tmp18 + tmp19 + tmp20 + tmp21 +
                      tmp4 * (2 * nx * px + tmp7) + (tmp14 + tmp22) * tmp9),
        2 * nx * tmp23, 2 * ny * tmp26,
        -4 * ny * q1y + tmp20 + tmp21 + 2 * tmp4 * (2 * ny * py + tmp6) +
            (4 * ny * px + tmp22) * tmp9 + 4 * ny * y,
        2 * (tmp27 + tmp28) * tmp29 -
            2 * tmp16 * (q1x - px * tmp4 + py * tmp9 - x),
        2 * (tmp24 + tmp25) * tmp29 + 2 * tmp11 * (q1y + tmp27 + tmp28 - y);

    Eigen::Matrix<csm_scalar, 3, 2> d2J_dx_dp;
    d2J_dx_dp << 2 * nx * tmp30, 2 * nx * tmp31, 2 * ny * tmp30, 2 * ny * tmp31,
        -2 * tmp11 * tmp30 + 2 * tmp29 * tmp31,
        2 * tmp16 * tmp31 + 2 * tmp29 * (-(nx * tmp4) - ny * tmp9);

    Eigen::Matrix<csm_scalar, 3, 2> d2J_dx_dq1;
    d2J_dx_dq1 << -2 * tmp0, tmp32, tmp32, -2 * tmp13, 2 * nx * tmp11,
        2 * ny * tmp11;

    Eigen::Matrix<csm_scalar, 2, 2> dn_dr;
    dn_dr << q2r * tmp35 * (-(q1r * tmp36) + q2r * tmp37) * tmp38,
        q1r * tmp35 * (q1r * tmp36 - q2r * tmp37) * tmp38,
        q2r * tmp35 * tmp38 * (-(q1r * tmp39) + q2r * tmp40),
        q1r * tmp35 * tmp38 * (q1r * tmp39 - q2r * tmp40);

    Eigen::Matrix<csm_scalar, 2, 1> dp_dr;
    dp_dr << std::cos(pa), std::sin(pa);

    Eigen::Matrix<csm_scalar, 2, 1> dq1_dr;
    dp_dr << tmp36, tmp39;

    d2J_dxdy2.col(i) = d2J_dx_dp * dp_dr;
    d2J_dxdy1.col(j1) += d2J_dx_dn * dn_dr.col(0) + d2J_dx_dq1 * dq1_dr;
    d2J_dxdy1.col(j2) += d2J_dx_dn * dn_dr.col(1);
  }

  d2J_dx2.triangularView<Eigen::StrictlyLower>() =
      d2J_dx2.triangularView<Eigen::StrictlyUpper>().transpose();
  matrix3 d2J_dx2inv = d2J_dx2.inverse();
  dx_dy1             = -1.0 * d2J_dx2inv * d2J_dxdy1;
  dx_dy2             = -1.0 * d2J_dx2inv * d2J_dxdy2;
  cov0_x             = dx_dy1 * dx_dy1.transpose() + dx_dy2 * dx_dy2.transpose();
#endif
}

// matrix compute_C_k(const vector &p_j1, const vector &p_j2)  {
//	vector d = p_j1 - p_j2;
//	csm_scalar alpha = M_PI/2 + atan2(d[1], d[0]);
//	csm_scalar c = std::cos(alpha); csm_scalar s = std::sin(alpha);
//	matrix v(2, 2);
//	v <<
//	    c*c, c*s,
//		c*s, s*s;
//	return v;
//}
//
//
// matrix dC_drho(const vector &p1, const vector &p2) {
//	csm_scalar eps = 0.001;
//
//	matrix C_k = compute_C_k(p1, p2);
//	vector p1b = p1 + p1.normalized() * eps;
//	matrix C_k_eps = compute_C_k(p1b,p2);
//	return (C_k_eps - C_k) / eps;
//}

} /* namespace csm */
