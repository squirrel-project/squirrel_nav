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



#ifndef FEATURESHOT_H
#define FEATURESHOT_H
#include <iostream>
#include "datatypes_squirrel.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/shot_omp.h>
#include "EstimateNormal_omp.h"

using namespace std;
class FeatureEstimationSHOT
{


 EstimateNormalOmp estimate_normal;
 pcl::SHOTEstimationOMP<Point, PointNormal, pcl::SHOT352> pfh;


 public:
  FeatureEstimationSHOT();


  void estimateFeature(const PointCloud::Ptr &cloud_1,const float normal_radius,const float feature_radius,pcl::PointCloud<pcl::SHOT352>::Ptr &feature, std::vector<int> &finite_points);

  void estimateFeature(const PointCloud::Ptr &cloud_1,const PointCloud::Ptr &search_surface,const float normal_radius,const float feature_radius,pcl::PointCloud<pcl::SHOT352>::Ptr &feature,std::vector<int> &finite_points);

void estimateFeature(const PointCloud::Ptr &scene,const NormalCloud::Ptr &cloud_normal,const std::vector <float> &feature_radius,pcl::PointCloud<pcl::SHOT352>::Ptr &feature, std::vector<int> &finite_points);

  void estimateFeature(const PointCloud::Ptr &cloud_1,const NormalCloud::Ptr &cloud_normal,const float feature_radius,pcl::PointCloud<pcl::SHOT352>::Ptr &feature, std::vector<int> &finite_points);
};

#include "FeatureSHOT.hpp"
#endif

