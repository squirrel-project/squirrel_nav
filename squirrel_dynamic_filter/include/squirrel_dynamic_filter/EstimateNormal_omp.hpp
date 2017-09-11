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
