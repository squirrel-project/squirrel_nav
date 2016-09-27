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

