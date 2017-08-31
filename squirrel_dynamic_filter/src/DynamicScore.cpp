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
///Used for calculating whether a point is static or dynamic by comparing the
//estimated motion with odometry of the robot

void DynamicFilter::DynamicScore(const PointCloud::Ptr &cloud,const bool is_first,const PointCloud::Ptr &score)
{
 arma::mat covariance(3,3);
 arma::vec mean(3);
 arma::vec observation(3);
 covariance(0,0) = 1.0; covariance(0,1) = 0.0; covariance(0,2) = 0.0;
 covariance(1,0) = 0.0; covariance(1,1) = 1.0; covariance(1,2) = 0.0;
 covariance(2,0) = 0.0; covariance(2,1) = 0.0; covariance(2,2) = 1.0;
 covariance *= 0.001;
 mean[0] = 0.0;
 mean[1] = 0.0;
 mean[2] = 0.0;
 observation[0] = 0.0;
 observation[1] = 0.0;
 observation[2] = 0.0;

 mlpack::distribution::GaussianDistribution dist(mean,covariance);
 float max = dist.Probability(observation);

 pcl::Correspondences all_correspondences;
 pcl::registration::CorrespondenceEstimation< Point, Point> est;

/// Associate points from t-1 to t to transfer the prior information
 if(!is_first)
 {
  est.setInputSource (cloud);
  est.setInputTarget (score);
  est.determineCorrespondences (all_correspondences);
 }
 else
  frame_1.prior_dynamic.assign(cloud->points.size(),0.2);//Points have a higher chance of being static if no previous information is available


 std::vector <float> current_belief(frame_1.motion_init.size(),0.0);
 float prior;
 #pragma parallel omp for num_threads(8)
 for(size_t i = 0; i < frame_1.motion_init.size(); ++i)
 {
  Isometry3D motion_diff = frame_1.motion_init[i].inverse() * odometry_diff;
  observation[0] = motion_diff(0,3);
  observation[1] = motion_diff(1,3);
  observation[2] = motion_diff(2,3);
  if(all_correspondences.empty())
   prior = frame_1.prior_dynamic[i];
  else
  {
   if(all_correspondences[i].distance <= 0.01 * 0.01 )
   {
    if(all_correspondences[i].index_match >= frame_1.prior_dynamic.size())
    {


      ROS_ERROR("%s:%ld,%ld,%ld",ros::this_node::getName().c_str(),frame_1.prior_dynamic.size(),score->points.size(),cloud->points.size());
     getchar();




    }
     prior = frame_1.prior_dynamic[all_correspondences[i].index_match];

   }
   else
    prior = 0.2;
  }



  float posterior_d = (1.0 - dist.Probability(observation)/max)  * ((p_d_d * prior) + (p_d_s * (1.0 - prior)));
  float posterior_s = (dist.Probability(observation)/max) * ((p_s_s * (1.0 - prior)) + (p_s_d * prior));
  float probability = posterior_d/(posterior_d + posterior_s);
  if(probability < 0.0)
  {

   ROS_ERROR("%s: %f, %f, %f, %f",ros::this_node::getName().c_str(),posterior_d,posterior_s,prior,dist.Probability(observation)/max);
   getchar();
  }
  if(probability > 1.0)
  {

   ROS_ERROR("%s: %f, %f, %f, %f",ros::this_node::getName().c_str(),posterior_d,posterior_s,prior,dist.Probability(observation)/max);

   getchar();

  }

  current_belief[i] = probability;
 }

 frame_1.prior_dynamic.clear();
 frame_1.prior_dynamic = current_belief;

}
