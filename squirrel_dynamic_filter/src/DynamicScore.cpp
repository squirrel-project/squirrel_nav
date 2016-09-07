#include "dynamic_filter_node.h"


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
 if(!is_first)
 {
  est.setInputSource (cloud);
  est.setInputTarget (score);
  est.determineCorrespondences (all_correspondences);
 }
 else
  frame_1.prior_dynamic.assign(cloud->points.size(),0.2); 
 
 
 std::vector <float> current_belief(frame_1.motion_init.size(),0.0);
 //fprintf(stderr,"error%d,%d,%d",frame_1.prior_dynamic.size(),score->points.size(),cloud->points.size());
 
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
   if(all_correspondences[i].distance <= 0.1)
   {
    if(all_correspondences[i].index_match >= frame_1.prior_dynamic.size())
    {


     fprintf(stderr,"error%d,%d,%d",frame_1.prior_dynamic.size(),score->points.size(),cloud->points.size());
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

   fprintf(stderr,"%f,%f,%f,%f\n", posterior_d,posterior_s,prior,dist.Probability(observation)/max);
   getchar(); 


  } 
  if(probability > 1.0)
  {

   fprintf(stderr,"greater than 1 %f,%f,%f,%f\n", posterior_d,posterior_s,prior,dist.Probability(observation)/max);
   getchar(); 


  } 

  current_belief[i] = probability;
 }

 frame_1.prior_dynamic.clear();
 frame_1.prior_dynamic = current_belief;

}
