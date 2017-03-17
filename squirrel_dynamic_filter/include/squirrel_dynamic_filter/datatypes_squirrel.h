// Copyright (c) 2016-2017, Ayush Dewan and Wolfram Burgard
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef DATATYPES_H
#define DATATYPES_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <ctime>

typedef std::chrono::duration <double> TimeDiff;
typedef std::chrono::system_clock SystemClock;
typedef std::chrono::time_point<std::chrono::system_clock> measure_time;
typedef pcl::PointXYZ Point;
typedef pcl::Normal PointNormal;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZI PointIntensity;
typedef pcl::SHOT352 PointSHOT;
typedef pcl::PointCloud <Point> PointCloud;
typedef pcl::PointCloud <PointNormal> NormalCloud;
typedef pcl::PointCloud <PointNormal>::Ptr NormalCloudPtr;
typedef pcl::PointCloud <PointIntensity> IntensityCloud;
typedef pcl::PointCloud <PointRGB> RGBCloud;
typedef pcl::PointCloud <PointSHOT> SHOTCloud;

#endif

