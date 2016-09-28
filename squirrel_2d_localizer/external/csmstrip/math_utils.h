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

#ifndef H_MATH_UTILS
#define H_MATH_UTILS

#include <string>
#include "scalar.h"

/* Sometimes I really don't understand compilers.. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef NAN
#include <limits>
#define NAN std::numeric_limits<csm_scalar>::quiet_NaN()
#endif

namespace csm {

/** Returns norm of 2D point p */
csm_scalar norm_d(const csm_scalar p[2]);

csm_scalar distance_d(const csm_scalar a[2], const csm_scalar b[2]);
csm_scalar distance_squared_d(const csm_scalar a[2], const csm_scalar b[2]);

/** Returns an angle difference in the [-pi, pi] range */
csm_scalar angleDiff(csm_scalar a, csm_scalar b);
csm_scalar square(csm_scalar x);

/** Degrees to radians */
csm_scalar deg2rad(csm_scalar deg);

/** Radians to degrees */
csm_scalar rad2deg(csm_scalar rad);

int minmax(int from, int to, int x);

/** Copies n doubles from from to to */
void copy_d(const csm_scalar* from, int n, csm_scalar* to);

/** These are the operators defined in Smith & Cheeseman  */
void ominus_d(const csm_scalar x[3], csm_scalar res[3]);
void oplus_d(const csm_scalar x1[3], const csm_scalar x2[3], csm_scalar res[3]);
void pose_diff_d(
    const csm_scalar second[3], const csm_scalar first[3], csm_scalar res[3]);

void transform_d(
    const csm_scalar point2d[2], const csm_scalar pose[3],
    csm_scalar result2d[2]);

/** Projects (p[0],p[1]) on the LINE passing through (ax,ay)-(bx,by). If
distance!=0, distance is set
to the distance from the point to the segment */
void projection_on_line_d(
    const csm_scalar a[2], const csm_scalar b[2], const csm_scalar p[2],
    csm_scalar res[2], csm_scalar* distance);

/** Projection of P on the SEGMENT A-B */
void projection_on_segment_d(
    const csm_scalar a[2], const csm_scalar b[2], const csm_scalar P[2],
    csm_scalar proj[2]);

/** Distance of x from its projection on segment a-b */
csm_scalar dist_to_segment_d(
    const csm_scalar a[2], const csm_scalar b[2], const csm_scalar x[2]);

/** Same thing as dist_to_segment_d(), but squared */
csm_scalar dist_to_segment_squared_d(
    const csm_scalar a[2], const csm_scalar b[2], const csm_scalar x[2]);

/* Executes ray tracing for a segment. p0 and p1 are the segments extrema, eye
is the position
of the eye, and direction is the direction of the ray coming out of the eye.
Returns true
if the ray intersects the segment, and in that case *range contains the length
of the ray. */
int segment_ray_tracing(
    const csm_scalar p0[2], const csm_scalar p1[2], const csm_scalar eye[2],
    csm_scalar direction, csm_scalar* range);

/** Returns the orientation of the normal for the line passing through p0-p1 */
csm_scalar segment_alpha(const csm_scalar p0[2], const csm_scalar p1[2]);

/** A function to print poses and covariances in a friendly way */
// const char* friendly_pose(const csm_scalar*pose);
std::string friendly_pose(const csm_scalar* pose);

/** Returns true v is NAN */
int is_nan(csm_scalar v);

/** Returns true if any value in d is NAN */
int any_nan(const csm_scalar* d, int n);

/** Count numbers of items in array v equal to value */
int count_equal(const int* v, int n, int value);

/** Normalizes an angle in the 0-2PI range */
csm_scalar normalize_0_2PI(csm_scalar angle);

/** Maximum value in the array */
csm_scalar max_in_array(const csm_scalar* v, int n);

} /* namespace csm */

#endif
