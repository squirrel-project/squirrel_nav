// KLDSampling.cpp --- 
// 
// Filename: KLDSampling.cpp
// Description: KLD sampling method
// Maintainer: Federico Bonairdi
// Created: Tue Feb 17 11:55:53 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//    ROS Hydro
//    ROS Indigo
// 

// Commentary:
// 
//  /***********************************************************************
//     KLD-SAMPLING: Adequately Sampling from an Unknown Distribution.
// 
//     Copyright (C) 2006 - Patrick Beeson  (pbeeson@cs.utexas.edu):
// 
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
// 
//     This program is distributed in the hope that it will be useful, but
//     WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//     General Public License for more details.
// 
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301
//     USA
//   ***********************************************************************/
//     

// Code:


#ifndef SQUIRREL_LOCALIZER_KLD_SAMPLING_H_
#define SQUIRREL_LOCALIZER_KLD_SAMPLING_H_

#include <vector>
#include <cstdlib>

using namespace std;

class kld_sampling {
  
public:
  kld_sampling();
  void init(float, float, const vector<float>&, int sample_min=absolute_min);
  int update(const vector<float>&);
  
private:
  static const int absolute_min=10;
  float confidence, max_error;
  vector<float> bin_size;

  static vector<float> ztable;
  int num_samples;
  vector< vector <float> > bins;

  vector<float> curr_sample;
  int support_samples, kld_samples;
  float zvalue;

  bool in_empty_bin();
  void build_table();
};

#endif  /* SQUIRREL_LOCALIZER_KLD_SAMPLING_H_ */

// 
// KLDSampling.h ends here
