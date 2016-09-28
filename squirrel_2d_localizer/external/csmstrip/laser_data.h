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

#ifndef H_LASER_DATA
#define H_LASER_DATA

#include <stdio.h>
#include <sys/time.h>

#include "restrict.h"
#include "scalar.h"

namespace csm {

struct correspondence;

typedef struct {
  csm_scalar p[2];
  csm_scalar rho, phi;
} point2d;

typedef struct { csm_scalar n[2]; } normal2d;

struct laser_data {
  int nrays;

  csm_scalar min_theta;
  csm_scalar max_theta;

  csm_scalar* restrict theta;

  int* restrict valid;
  csm_scalar* restrict readings;

  int* restrict cluster;

  csm_scalar* restrict alpha;
  csm_scalar* restrict cov_alpha;
  int* restrict alpha_valid;

  csm_scalar* restrict readings_sigma;

  csm_scalar* restrict true_alpha;

  struct correspondence* restrict corr;

  csm_scalar true_pose[3];
  csm_scalar odometry[3];
  csm_scalar estimate[3];

  /** Cartesian representation */
  point2d* restrict points;
  /** Cartesian representation, in "world" (laser_ref) coordinates.
      Computed using ld_compute_world_coords() */
  point2d* restrict points_w;
  /** Normals for each point */
  normal2d* restrict normals;

  /** Timestamp */
  // struct timeval tv;
  // char hostname[32];

  /* Jump tables needed by find_correspondences_tricks(). */
  int *restrict up_bigger, *restrict up_smaller, *restrict down_bigger,
      *restrict down_smaller;
};

struct correspondence {
  /** 1 if this correspondence is valid  */
  int valid;
  /** Closest point in the other scan.  */
  int j1;
  /** Second closest point in the other scan.  */
  int j2;
  /** Type of correspondence (point to point, or point to line) */
  enum { corr_pp = 0, corr_pl = 1 } type;
  /** Squared distance from p(i) to point j1 */
  csm_scalar dist2_j1;
};

typedef struct laser_data* LDP;

/** This returns a new structure, with all fields initialized */
LDP ld_alloc_new(int nrays);

/** This DOES free() the pointer  */
void ld_free(LDP);

/** This allocs the fields in the given structure. Use ld_alloc_new(), not this.
 */
void ld_alloc(LDP, int nrays);

/** This does NOT free the pointer. Don't use -- use ld_alloc_new()/ld_free()
 * instead. */
void ld_dealloc(LDP);

/** Fills the x,y fields in "points" by transforming (theta, reading) to
 * cartesian */
void ld_compute_cartesian(LDP);

/** Computes the "points_w" coordinates by roto-translating "points" */
void ld_compute_world_coords(LDP, const csm_scalar* pose);

/** Fills the fields: *up_bigger, *up_smaller, *down_bigger, *down_smaller.*/
void ld_create_jump_tables(LDP);

/** Computes an hash of the correspondences */
unsigned int ld_corr_hash(LDP);

/** Returns the number of valid correspondences. */
int ld_num_valid_correspondences(LDP);

/** Do an extensive sanity check about the data contained in the structure. */
int ld_valid_fields(LDP);

/** A simple clustering algorithm. Sets the `cluster' field in the structure. */
void ld_simple_clustering(LDP ld, csm_scalar threshold);

/** A cool orientation estimation algorithm. Needs cluster. */
void ld_compute_orientation(LDP ld, int size_neighbourhood, csm_scalar sigma);

/**
        Tries to read a laser scan from file. If error or EOF, it returns 0.
        Whitespace is skipped. If first valid char is '{', it tries to read
        it as JSON. If next char is 'F' (first character of "FLASER"),
        it tries to read in Carmen format. Else, 0 is returned.
*/
LDP ld_read_smart(FILE*);

/**
        Tries to read a laser scan from a string.
*/
LDP ld_read_smart_string(const char*);

/** Read next FLASER line in file (initializes ld).
        Returns 0 on failure. If the file is EOF, it returns 1
        and sets ld to 0.
        You probably want to use the ld_read_smart() function. */
int ld_read_next_laser_carmen(FILE*, LDP* ld);

/** Read laser data from a Carmen-formatted line */
LDP ld_from_carmen_string(const char* line);

/** Reads all the scans it can find. */
int ld_read_all(FILE* file, LDP** array, int* num);

/** Read a scan every interval (1=all)*/
int ld_read_some_scans(FILE* file, LDP** array, int* num, int interval);

/** Write a scan in carmen format */
void ld_write_as_carmen(LDP ld, FILE* stream);

/** Write a scan according to out_format = {"carmen", "json"} */
void ld_write_format(LDP ld, FILE* stream, const char* out_format);

void possible_interval(
    const csm_scalar* p_i_w, LDP laser_sens,
    csm_scalar max_angular_correction_deg, csm_scalar max_linear_correction,
    int* from, int* to, int* start_cell);

} /* namespace csm */

#include "laser_data_inline.h"

#endif
