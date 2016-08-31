#ifndef EstimateNormal_omp_H
#define EstimateNormal_omp_H
//#ifdef PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include "datatypes_squirrel.h"
using namespace std;


class EstimateNormalOmp
{
public:
EstimateNormalOmp();
template <class PointType,class NormalType> void cal_normal(boost::shared_ptr<pcl::PointCloud<PointType> > cloud_input,boost::shared_ptr<pcl::PointCloud<NormalType> >normal,boost::shared_ptr<pcl::PointCloud<PointType> > search_surface,float search_radius,int number_neighbours);

template <class PointType,class NormalType> void cal_normal(boost::shared_ptr<pcl::PointCloud<PointType> > cloud_input,boost::shared_ptr<pcl::PointCloud<NormalType> >normal,boost::shared_ptr<pcl::PointCloud<PointType> > search_surface,float search_radius,int number_neighbours,std::vector<int> &indices);

};


#include"EstimateNormal_omp.hpp"
//#endif
#endif

