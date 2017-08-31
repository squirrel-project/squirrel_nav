// The MIT License (MIT)
//
// Copyright (c) 2016-2017 Ayush Dewan and Wolfram Burgard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.



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

