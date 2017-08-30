#ifndef EstimateNormal_omp_HPP
#define EstimateNormal_omp_HPP

#include"EstimateNormal_omp.h"

inline EstimateNormalOmp::EstimateNormalOmp()
{

}

template<class PointType,class NormalType> void EstimateNormalOmp::cal_normal(boost::shared_ptr<pcl::PointCloud<PointType> > cloud_input,boost::shared_ptr<pcl::PointCloud<NormalType> >normal,boost::shared_ptr<pcl::PointCloud<PointType> > search_surface,float search_radius,int number_neighbours)
{
	pcl::NormalEstimationOMP<PointType,NormalType>normal_estimation;
	normal_estimation.setInputCloud (cloud_input);
	boost::shared_ptr<pcl::search::KdTree<PointType> > tree (new pcl::search::KdTree<PointType>);
	normal_estimation.setSearchMethod (tree);
 normal_estimation.setSearchSurface (search_surface);
 if(number_neighbours==0)
  normal_estimation.setRadiusSearch (search_radius);
 else
  normal_estimation.setKSearch (number_neighbours);

 normal_estimation.setViewPoint(0.0,0.0,4.0);//0.0,0.0,4.0
 normal_estimation.compute (*normal);


}


template<class PointType,class NormalType> void EstimateNormalOmp::cal_normal(boost::shared_ptr<pcl::PointCloud<PointType> > cloud_input,boost::shared_ptr<pcl::PointCloud<NormalType> >normal,boost::shared_ptr<pcl::PointCloud<PointType> > search_surface,float search_radius,int number_neighbours,std::vector<int>&indices)
{
	pcl::NormalEstimationOMP<PointType,NormalType>normal_estimation;
	normal_estimation.setInputCloud (cloud_input);
	boost::shared_ptr<pcl::search::KdTree<PointType> > tree (new pcl::search::KdTree<PointType>);
	normal_estimation.setSearchMethod (tree);
 normal_estimation.setSearchSurface (search_surface);
 if(number_neighbours==0)
  normal_estimation.setRadiusSearch (search_radius);
 else
  normal_estimation.setKSearch (number_neighbours);

 normal_estimation.setViewPoint(0.0,0.0,4.0);//0.0,0.0,4.0
 normal_estimation.compute (*normal);


 int iter = 0;
 for(auto i:normal->points)
 {
  if(pcl::isFinite(i))
   indices.push_back(iter);

  iter+=1;
 }
 


}

#endif
