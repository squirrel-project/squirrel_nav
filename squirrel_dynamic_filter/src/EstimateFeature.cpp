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



#include "dynamic_filter_node.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>



///sampling step, take a point calculate its small neighbourhood and then that
//point and then calculate descriptor for only that point and not for the
//neighbourhood
void DynamicFilter::sample(const Frame frame,const float radius, const float threshold,std::vector <int> &sampled_finite)
{
 pcl::search::KdTree <Point> kdtree_point;
 kdtree_point.setInputCloud (frame.raw_input);
 std::vector <bool> check(frame.raw_input->points.size(),false);



 int counter = 0;
 for(auto &point:frame.raw_input->points)
 {

  if(check[counter])
  {
   counter+=1;
   continue;
  }
  sampled_finite.push_back(counter) ;
  std::vector <int> n_indices;
  std::vector <float> n_distances;
  if (kdtree_point.radiusSearch (point,radius,n_indices,n_distances) > 0 )
  {
   check[counter] = true;
   for(size_t i = 0; i < n_indices.size(); ++i)
   {

    if(sqrt(n_distances[i]) < threshold)
    {


     check[n_indices[i]] = true;
    }

   }
  }

  counter+=1;


 }

}

////Features are not calculated for every point, but for a subset of points.The first step is to sample pointcloud with the inituition that very
//close by points will have similar feature descriptor and then calculate
//descriptors for sampled points only. This makes things fatser since
//calculating feature for every point takes more time.
void DynamicFilter::EstimateFeature(Frame &frame)
{
 std::vector <int> sampled_indices;
 sample(frame,feature_radius,0.009,sampled_indices);
 PointCloud::Ptr sampled(new PointCloud);
 pcl::copyPointCloud(*frame.raw_input,sampled_indices,*sampled);
 std::vector <int> finite_points;
 FeatureEstimationSHOT FeatureEstimate;
 SHOTCloud::Ptr scene_feature (new SHOTCloud);
 FeatureEstimate.estimateFeature(sampled,frame.raw_input,normal_radius,feature_radius,scene_feature,finite_points);

 for(auto &index:finite_points)
  frame.finite_points.push_back(sampled_indices[index]);

 pcl::copyPointCloud(*frame.raw_input,frame.finite_points,*frame.cloud_input);
 pcl::copyPointCloud(*scene_feature,finite_points,*frame.feature);

}



void DynamicFilter::sample_dynamic(const PointCloud::Ptr dynamic ,const float radius, const float threshold,std::vector <int> &sampled_finite)
{
 pcl::search::KdTree <Point> kdtree_point;
 kdtree_point.setInputCloud (dynamic);
 std::vector <bool> check(dynamic->points.size(),false);



 int counter = 0;
 for(auto &point:dynamic->points)
 {

  if(check[counter])
  {
   counter+=1;
   continue;
  }
  sampled_finite.push_back(counter) ;
  std::vector <int> n_indices;
  std::vector <float> n_distances;
  if (kdtree_point.radiusSearch (point,radius,n_indices,n_distances) > 0 )
  {
   check[counter] = true;
   for(size_t i = 0; i < n_indices.size(); ++i)
   {

    if(sqrt(n_distances[i]) < threshold)
    {


     check[n_indices[i]] = true;
    }

   }
  }

  counter+=1;


 }

}

///Estimate feature for dynamic points.
void DynamicFilter::EstimateFeature(Frame &frame,const std::vector <int> &dynamic_indices)
{

 PointCloud::Ptr dynamic(new PointCloud);
 pcl::copyPointCloud(*frame.raw_input,dynamic_indices,*dynamic);

 std::vector <int> sampled_indices;
 sample_dynamic(dynamic,feature_radius,0.009,sampled_indices);
 PointCloud::Ptr sampled(new PointCloud);
 pcl::copyPointCloud(*dynamic,sampled_indices,*sampled);
 std::vector <int> finite_points;
 FeatureEstimationSHOT FeatureEstimate;
 SHOTCloud::Ptr scene_feature (new SHOTCloud);
 FeatureEstimate.estimateFeature(sampled,frame.raw_input,normal_radius,feature_radius,scene_feature,finite_points);

 for(auto &index:finite_points)
  frame.finite_points.push_back(dynamic_indices[sampled_indices[index]]);

 pcl::copyPointCloud(*frame.raw_input,frame.finite_points,*frame.cloud_input);
 pcl::copyPointCloud(*scene_feature,finite_points,*frame.feature);

}










