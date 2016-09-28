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

#include <assert.h>
#include <cmath>
#include "csm.h"

namespace csm {

int minmax(int from, int to, int x) {
  return (std::max)((std::min)(x, to), from);
}

void possible_interval(
    const csm_scalar* p_i_w, LDP ld, csm_scalar max_angular_correction_deg,
    csm_scalar max_linear_correction, int* from, int* to, int* start_cell) {
  csm_scalar angle_res = (ld->max_theta - ld->min_theta) / ld->nrays;

  /* Delta for the angle */
  csm_scalar delta = std::abs(deg2rad(max_angular_correction_deg)) +
                     std::abs(std::atan(max_linear_correction / norm_d(p_i_w)));

  /* Dimension of the cell range */
  int range = (int)std::ceil(delta / angle_res);

  /* To be turned into an interval of cells */
  csm_scalar start_theta = std::atan2(p_i_w[1], p_i_w[0]);

  /* Make sure that start_theta is in the interval [min_theta,max_theta].
     For example, -1 is not in [0, 2pi] */
  if (start_theta < ld->min_theta)
    start_theta += 2 * M_PI;
  if (start_theta > ld->max_theta)
    start_theta -= 2 * M_PI;

  *start_cell =
      (int)((start_theta - ld->min_theta) / (ld->max_theta - ld->min_theta) * ld->nrays);

  *from = minmax(0, ld->nrays - 1, *start_cell - range);
  *to   = minmax(0, ld->nrays - 1, *start_cell + range);

  if (0)
    printf(
        "from: %d to: %d delta: %f start_theta: %f min/max theta: [%f,%f] "
        "range: %d start_cell: %d\n",
        *from, *to, delta, start_theta, ld->min_theta, ld->max_theta, range,
        *start_cell);
}

csm_scalar distance_squared_d(const csm_scalar a[2], const csm_scalar b[2]) {
  csm_scalar x = a[0] - b[0];
  csm_scalar y = a[1] - b[1];
  return x * x + y * y;
}

csm_scalar distance_d(const csm_scalar a[2], const csm_scalar b[2]) {
  return std::sqrt(distance_squared_d(a, b));
}

int is_nan(csm_scalar v) { return v == v ? 0 : 1; }

int any_nan(const csm_scalar* d, int n) {
  int i;
  for (i = 0; i < n; i++)
    if (is_nan(d[i]))
      return 1;
  return 0;
}

csm_scalar norm_d(const csm_scalar p[2]) {
  return std::sqrt(p[0] * p[0] + p[1] * p[1]);
}

csm_scalar deg2rad(csm_scalar deg) { return deg * (M_PI / 180); }

csm_scalar rad2deg(csm_scalar rad) { return rad * (180 / M_PI); }

void copy_d(const csm_scalar* from, int n, csm_scalar* to) {
  int i;
  for (i  = 0; i < n; i++)
    to[i] = from[i];
}

void ominus_d(const csm_scalar x[3], csm_scalar res[3]) {
  csm_scalar c = std::cos(x[2]);
  csm_scalar s = std::sin(x[2]);
  res[0]       = -c * x[0] - s * x[1];
  res[1]       = s * x[0] - c * x[1];
  res[2]       = -x[2];
}

/** safe if res == x1 */
void oplus_d(
    const csm_scalar x1[3], const csm_scalar x2[3], csm_scalar res[3]) {
  csm_scalar c     = std::cos(x1[2]);
  csm_scalar s     = std::sin(x1[2]);
  csm_scalar x     = x1[0] + c * x2[0] - s * x2[1];
  csm_scalar y     = x1[1] + s * x2[0] + c * x2[1];
  csm_scalar theta = x1[2] + x2[2];
  res[0]           = x;
  res[1]           = y;
  res[2]           = theta;
}

void transform_d(
    const csm_scalar point2d[2], const csm_scalar pose[3],
    csm_scalar result2d[2]) {
  csm_scalar theta = pose[2];
  csm_scalar c     = std::cos(theta);
  csm_scalar s     = std::sin(theta);
  result2d[0]      = pose[0] + c * point2d[0] - s * point2d[1];
  result2d[1]      = pose[1] + s * point2d[0] + c * point2d[1];
}

void pose_diff_d(
    const csm_scalar pose2[3], const csm_scalar pose1[3], csm_scalar res[3]) {
  csm_scalar temp[3];
  ominus_d(pose1, temp);
  oplus_d(temp, pose2, res);

  while (res[2] > +M_PI)
    res[2] -= 2 * M_PI;
  while (res[2] < -M_PI)
    res[2] += 2 * M_PI;
}

csm_scalar square(csm_scalar x) { return x * x; }

csm_scalar angleDiff(csm_scalar a, csm_scalar b) {
  csm_scalar t = a - b;
  while (t < -M_PI)
    t += 2 * M_PI;
  while (t > M_PI)
    t -= 2 * M_PI;
  return t;
}

void projection_on_line_d(
    const csm_scalar a[2], const csm_scalar b[2], const csm_scalar p[2],
    csm_scalar res[2], csm_scalar* distance) {
  csm_scalar t0       = a[0] - b[0];
  csm_scalar t1       = a[1] - b[1];
  csm_scalar one_on_r = 1 / sqrt(t0 * t0 + t1 * t1);
  /* normal */
  csm_scalar nx = t1 * one_on_r;
  csm_scalar ny = -t0 * one_on_r;
  csm_scalar c = nx, s = ny;
  csm_scalar rho = c * a[0] + s * a[1];

  res[0] = c * rho + s * s * p[0] - c * s * p[1];
  res[1] = s * rho - c * s * p[0] + c * c * p[1];

  if (distance)
    *distance = std::fabs(rho - (c * p[0] + s * p[1]));
}

void projection_on_segment_d(
    const csm_scalar a[2], const csm_scalar b[2], const csm_scalar x[2],
    csm_scalar proj[2]) {
  projection_on_line_d(a, b, x, proj, 0);
  if ((proj[0] - a[0]) * (proj[0] - b[0]) +
          (proj[1] - a[1]) * (proj[1] - b[1]) <
      0) {
    /* the projection is inside the segment */
  } else if (distance_squared_d(a, x) < distance_squared_d(b, x))
    copy_d(a, 2, proj);
  else
    copy_d(b, 2, proj);
}

csm_scalar dist_to_segment_squared_d(
    const csm_scalar a[2], const csm_scalar b[2], const csm_scalar x[2]) {
  csm_scalar projection[2];
  projection_on_segment_d(a, b, x, projection);
  csm_scalar distance_sq_d = distance_squared_d(projection, x);
  return distance_sq_d;
}

csm_scalar dist_to_segment_d(
    const csm_scalar a[2], const csm_scalar b[2], const csm_scalar x[2]) {
  csm_scalar proj[2];
  csm_scalar distance;
  projection_on_line_d(a, b, x, proj, &distance);
  if ((proj[0] - a[0]) * (proj[0] - b[0]) +
          (proj[1] - a[1]) * (proj[1] - b[1]) <
      0) {
    /* the projection is inside the segment */
    return distance;
  } else
    return std::sqrt(
        (std::min)(distance_squared_d(a, x), distance_squared_d(b, x)));
}

int count_equal(const int* v, int n, int value) {
  int num = 0, i;
  for (i = 0; i < n; i++)
    if (value == v[i])
      num++;
  return num;
}

csm_scalar normalize_0_2PI(csm_scalar t) {
  if (is_nan(t)) {
    sm_error("Passed NAN to normalize_0_2PI().\n");
    return std::numeric_limits<csm_scalar>::quiet_NaN();
  }
  while (t < 0)
    t += 2 * M_PI;
  while (t >= 2 * M_PI)
    t -= 2 * M_PI;
  return t;
}

csm_scalar dot_d(const csm_scalar p[2], const csm_scalar q[2]);

csm_scalar dot_d(const csm_scalar p[2], const csm_scalar q[2]) {
  return p[0] * q[0] + p[1] * q[1];
}

/* Executes ray tracing for a segment. p0 and p1 are the segments extrema, eye
is the position
of the eye, and direction is the direction of the ray coming out of the eye.
Returns true
if the ray intersects the segment, and in that case *range contains the length
of the ray. */
int segment_ray_tracing(
    const csm_scalar p0[2], const csm_scalar p1[2], const csm_scalar eye[2],
    csm_scalar direction, csm_scalar* range) {

  *range = std::numeric_limits<csm_scalar>::quiet_NaN();

  // p0 - p1
  csm_scalar arrow[2] = {p1[0] - p0[0], p1[1] - p0[1]};
  // Normal to segment line
  csm_scalar S[2] = {-arrow[1], arrow[0]};
  // Viewing direction
  csm_scalar N[2] = {std::cos(direction), std::sin(direction)};
  // If S*N = 0 then they cannot cross
  csm_scalar S_dot_N = dot_d(S, N);
  if (S_dot_N == 0)
    return 0;
  // Rho of the line in polar coordinates (multiplied by |S|)
  csm_scalar line_rho = dot_d(p0, S);
  // Rho of the eye  (multiplied by |S|)
  csm_scalar eye_rho = dot_d(eye, S);
  // Black magic
  csm_scalar dist = (line_rho - eye_rho) / S_dot_N;
  if (dist <= 0)
    return 0;

  // Now we check whether the crossing point
  // with the line lies within the segment

  // Crossing point
  csm_scalar crossing[2] = {eye[0] + N[0] * dist, eye[1] + N[1] * dist};
  // Half of the segment
  csm_scalar midpoint[2] = {csm_scalar(0.5) * (p1[0] + p0[0]),
                            csm_scalar(0.5) * (p1[1] + p0[1])};

  csm_scalar seg_size         = distance_d(p0, p1);
  csm_scalar dist_to_midpoint = distance_d(crossing, midpoint);

  if (dist_to_midpoint > seg_size / 2)
    return 0;

  *range = dist;
  return 1;
}

csm_scalar segment_alpha(const csm_scalar p0[2], const csm_scalar p1[2]) {
  csm_scalar arrow[2] = {p1[0] - p0[0], p1[1] - p0[1]};
  // Normal to segment line
  csm_scalar S[2] = {-arrow[1], arrow[0]};
  return std::atan2(S[1], S[0]);
}

// static char tmp_buf[1024];
// const char* friendly_pose(const csm_scalar*pose) {
std::string friendly_pose(const csm_scalar* pose) {
  static char tmp_buf[1024];
  sprintf(
      tmp_buf, "(%4.2f mm, %4.2f mm, %4.4f deg)", 1000 * pose[0],
      1000 * pose[1], rad2deg(pose[2]));
  return tmp_buf;
}

csm_scalar max_in_array(const csm_scalar* v, int n) {
  assert(n > 0);
  csm_scalar m = v[0];
  int i;
  for (i = 0; i < n; i++)
    if (v[i] > m)
      m = v[i];
  return m;
}

} /* namespace csm */
