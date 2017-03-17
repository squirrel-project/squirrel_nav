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
