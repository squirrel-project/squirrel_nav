#ifndef FEATURESHOT_HPP
#define FEATURESHOT_HPP

#include "FeatureSHOT.h"
#include <chrono>
inline FeatureEstimationSHOT::FeatureEstimationSHOT()
{




};

inline void FeatureEstimationSHOT::estimateFeature(const PointCloud::Ptr &scene,const float normal_radius,const float feature_radius,pcl::PointCloud<pcl::SHOT352>::Ptr &feature, std::vector<int> &finite_points)
{

 NormalCloud::Ptr cloud_normal_temp(new NormalCloud);
 NormalCloud::Ptr cloud_normal(new NormalCloud);
/*
 PointCloudGPU cloud_gpu,normal_gpu;
 cloud_gpu.upload(scene->points);
 EstimateNormalGPU estimate_normal;
 estimate_normal.cal_normal(cloud_gpu,normal_gpu,normal_radius,30);
 PointCloud cloud_point;
	normal_gpu.download(cloud_point.points);


	for( size_t i = 0 ; i < cloud_point.points.size(); ++i)
	{
		PointNormal normal;
		normal.normal_x = cloud_point.points[i].x;
		normal.normal_y = cloud_point.points[i].y;
		normal.normal_z = cloud_point.points[i].z;
		cloud_normal->points.push_back(normal);
	}
*/

 estimate_normal.cal_normal<Point,PointNormal>(scene,cloud_normal_temp,scene,normal_radius,0);

 PointCloud::Ptr scene_temp(new PointCloud);
 std::vector <int> finite_normal_pts;
 for(size_t i = 0; i < scene->points.size(); ++i)
 {

  if(pcl::isFinite(cloud_normal_temp->points[i]))
  {  
   finite_normal_pts.push_back(i);
   scene_temp->points.push_back(scene->points[i]);
   cloud_normal->points.push_back(cloud_normal_temp->points[i]);

  }
 
 }

scene->points.clear();
 
scene->points = scene_temp->points;

   std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    
 
// fprintf(stderr,"%d,%d",scene_temp->points.size(),scene->points.size());
// getchar();



 pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
 pfh.setNumberOfThreads(8);
 pfh.setInputCloud (scene);
 pfh.setInputNormals (cloud_normal);
 pfh.setSearchMethod (tree);
 pfh.setSearchSurface (scene);
 pfh.setRadiusSearch (feature_radius);
 pfh.compute (*feature);
 for(size_t j = 0; j < scene->points.size(); ++j)
 {
  if(pcl_isfinite(feature->points[j].descriptor[0]))
  {
   finite_points.push_back(j);
  }
 }
end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;


 cout << "normal : " << elapsed_seconds.count() << endl;


}

inline void FeatureEstimationSHOT::estimateFeature(const PointCloud::Ptr &scene,const PointCloud::Ptr &search_surface,const float normal_radius,const float feature_radius,pcl::PointCloud<pcl::SHOT352>::Ptr &feature,std::vector<int> &finite_points)
{
 NormalCloud::Ptr cloud_normal_temp(new NormalCloud);
 NormalCloud::Ptr cloud_normal(new NormalCloud);
 estimate_normal.cal_normal<Point,PointNormal>(search_surface,cloud_normal_temp,search_surface,normal_radius,0);
 PointCloud::Ptr scene_temp(new PointCloud);
 std::vector <int> normal_indices;

 for(size_t i = 0; i < search_surface->points.size(); ++i)
 {

  if(pcl::isFinite(cloud_normal_temp->points[i]))
  {
   scene_temp->points.push_back(search_surface->points[i]);
   cloud_normal->points.push_back(cloud_normal_temp->points[i]);
   //normal_indices.push_back(i);

  }
 
 }
// search_surface->points.clear();
// search_surface->points = scene_temp->points;

 pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
 pfh.setNumberOfThreads(8);
 pfh.setInputCloud (scene);
 pfh.setInputNormals (cloud_normal);
 pfh.setSearchMethod (tree);
 pfh.setSearchSurface (scene_temp);
 pfh.setRadiusSearch (feature_radius);
 pfh.compute (*feature);
 for(size_t j = 0; j < feature->points.size(); ++j)
 {
  if(pcl_isfinite(feature->points[j].descriptor[0]))
  {
   finite_points.push_back(j);
  }
 }
}



















inline void FeatureEstimationSHOT::estimateFeature(const PointCloud::Ptr &scene,const NormalCloud::Ptr &cloud_normal,const float feature_radius,pcl::PointCloud<pcl::SHOT352>::Ptr &feature, std::vector<int> &finite_points)
{

 pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
 pfh.setNumberOfThreads(8);
 pfh.setInputCloud (scene);
 pfh.setInputNormals (cloud_normal);
 pfh.setSearchMethod (tree);
 pfh.setSearchSurface (scene);
 pfh.setRadiusSearch (feature_radius);
 pfh.compute (*feature);
 for(size_t j = 0; j < scene->points.size(); ++j)
 {
  if(pcl_isfinite(feature->points[j].descriptor[0]))
  {
   finite_points.push_back(j);
  }
 }

}

inline void FeatureEstimationSHOT::estimateFeature(const PointCloud::Ptr &scene,const NormalCloud::Ptr &cloud_normal,const std::vector <float> &feature_radius,pcl::PointCloud<pcl::SHOT352>::Ptr &feature, std::vector<int> &finite_points)
{

  feature->points.resize(scene->points.size());

#pragma omp parallel for
 for(size_t j = 0; j < scene->points.size(); ++j)
 {
 
  PointCloud::Ptr cloud(new PointCloud);

  pcl::PointCloud < pcl::SHOT352 >::Ptr cloud_feature(new pcl::PointCloud < pcl::SHOT352 >);

  cloud->points.push_back(scene->points[j]);

  pcl::SHOTEstimationOMP<Point, PointNormal, pcl::SHOT352> pfh;
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pfh.setNumberOfThreads(1);
  pfh.setSearchMethod (tree);
  pfh.setSearchSurface (scene);
  pfh.setInputNormals (cloud_normal);
  pfh.setInputCloud (cloud);
  pfh.setRadiusSearch (feature_radius[j]);
  if(feature_radius[j] > 0.0)
  {
   pfh.compute (*cloud_feature);
   feature->points[j] = cloud_feature->points[0];
  }
  else
   feature->points[j].descriptor[0] = std::numeric_limits<float>::quiet_NaN();



 }
 for(size_t j = 0; j < scene->points.size(); ++j)
 { 
  if(pcl_isfinite(feature->points[j].descriptor[0]))
   finite_points.push_back(j);
 }


}
#endif




