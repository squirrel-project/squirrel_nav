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

