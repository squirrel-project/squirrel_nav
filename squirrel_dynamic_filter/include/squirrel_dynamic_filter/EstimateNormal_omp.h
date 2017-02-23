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

