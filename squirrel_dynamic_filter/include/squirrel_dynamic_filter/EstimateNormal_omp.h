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

