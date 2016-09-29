#include "dynamic_filter_node.h"
using namespace std;

DynamicFilter::DynamicFilter()
{
 n_.getParam("/DownSamplingRadius",down_sampling_radius);
 n_.getParam("/FeatureClusterMaxLength",feature_cluster_max_length);
 n_.getParam("/FeatureClusterMaxPoints",feature_cluster_max_points);
 n_.getParam("/FeatureRadius",feature_radius);
 n_.getParam("/FeatureScoreThreshold",feature_score_threshold);
 n_.getParam("/FilterRadius",filter_radius);
 n_.getParam("/FilterVariance",filter_variance);
 n_.getParam("/KeyPointRadius",keypoint_radius);
 n_.getParam("/NormalRadius",normal_radius);
 n_.getParam("/OutputFolder",output_folder);
 n_.getParam("/MaxMotion",max_motion);
 n_.getParam("/SamplingRadius",sampling_radius);
 n_.getParam("/Verbose",is_verbose);
 n_.getParam("/StoreResults",store_results);

 ss.str("");
 ss << output_folder << "parameteres.csv";
 ofstream myfile_params(ss.str().c_str());

 myfile_params << down_sampling_radius << endl << feature_cluster_max_length << endl << feature_cluster_max_points << endl << feature_radius << endl << feature_score_threshold << endl << filter_radius << endl << filter_variance << endl << keypoint_radius << endl << normal_radius << endl << max_motion << endl << sampling_radius;

 myfile_params.close();

 ss.str("");
 ss << output_folder << "odometry.csv";
 myfile_odom.open(ss.str().c_str());

 ss.str("");
 ss << output_folder << "time.csv";
 time_write.open(ss.str().c_str());
 
 dynamic_filter_service = n_.advertiseService("dynamic_filter",&DynamicFilter::DynamicFilterSrvCallback,this);



}


bool DynamicFilter::DynamicFilterSrvCallback(squirrel_dynamic_filter_msgs::DynamicFilterSrv::Request &req,squirrel_dynamic_filter_msgs::DynamicFilterSrv::Response &res)
{

 ///If frame are skipped then reinitailze frame_1
 if(!is_first_frame)
 {

  if(req.frame_id != frame_1.frame_id + 1)
   is_first_frame = true;

 }
 if(is_first_frame)
 {
  frame_1.clear();
  pcl::fromROSMsg(req.cloud,*frame_1.raw_input);
  Vector7d odometry;
  odometry[0] = req.odometry[0];
  odometry[1] = req.odometry[1];
  odometry[2] = req.odometry[2];
  odometry[3] = req.odometry[3];
  odometry[4] = req.odometry[4];
  odometry[5] = req.odometry[5];
  odometry[6] = req.odometry[6];
  frame_1.odometry = g2o::internal::fromVectorQT(odometry); 	
  frame_1.frame_id = req.frame_id;
  EstimateFeature(frame_1);
  is_first_frame = false;
  if(is_verbose)
   ROS_INFO("first frame %s:%ld,%d",ros::this_node::getName().c_str(),frame_1.raw_input->points.size(),frame_1.frame_id);
 }
 else
 {

  start_total = SystemClock::now(); 
  frame_2.clear();
  pcl::fromROSMsg(req.cloud,*frame_2.raw_input);
  Vector7d odometry;
  odometry[0] = req.odometry[0];
  odometry[1] = req.odometry[1];
  odometry[2] = req.odometry[2];
  odometry[3] = req.odometry[3];
  odometry[4] = req.odometry[4];
  odometry[5] = req.odometry[5];
  odometry[6] = req.odometry[6];
  frame_2.odometry = g2o::internal::fromVectorQT(odometry); 	
  frame_2.frame_id = req.frame_id;
  odometry_diff = frame_2.odometry.inverse() * frame_1.odometry; 
  if(is_verbose)
   ROS_INFO("second frame %s:%ld,%d",ros::this_node::getName().c_str(),frame_2.raw_input->points.size(),frame_2.frame_id);

  std::vector <int> index_query;
  std::vector <int> index_match;
  std::vector <int> indices_dynamic;

///prior_dynamic tells no static info available and the usual shit
    //if both are non-empty then sme things are detected by static and some are
    //detected for dynamic  1 1
    //if index_query is non-empty and other is empty do nothing.1 0
    //if index_query is empty and other is not then do everything 0 1
    //if both are empty do everything. Both will only be empty if prior_dynamic
    //is empty  
  start = SystemClock::now(); 

////if previous information exists then use it for estimating correspondences
//using eucldean distance for static points. Mainly used to increase the speed
//by avoiding feature calculation for static points and using nearest neighbour
//for static

  if(!frame_1.prior_dynamic.empty())
   EstimateCorrespondenceEuclidean(0.02,index_query,index_match,indices_dynamic);
  end = SystemClock::now();
  time_diff = end - start;
  correspondence_time = time_diff.count();


/*

  
  ss.str("");
  ss << output_folder << "query_a_" << frame_1.frame_id << ".csv";
  ofstream myfile_query(ss.str().c_str());

  ss.str("");
  ss << output_folder << "match_a_" << frame_1.frame_id << ".csv";
  ofstream myfile_match(ss.str().c_str());
  
  for(size_t i = 0; i < index_query.size(); ++i)
  {
   myfile_query << index_query[i] << endl;
   myfile_match << index_match[i] << endl;
  }

  myfile_query.close();
  myfile_match.close();

*/

  
///if no previous information available or they are no dynamic point calculate
//feature for correspondences

  if(frame_1.prior_dynamic.empty() || !indices_dynamic.empty())
  {
  ///Estimate feature for all the points as there is no static points 
   if(frame_1.prior_dynamic.empty()|| index_query.empty())
    {
     EstimateFeature(frame_2);////Estimate features
  
     EstimateCorrespondencePoint(max_motion,sampling_radius,2,index_query,index_match);///Estimate correspondences
 
    }
   ///Estimate feature for the dynamic points
   else if(!index_query.empty() && !indices_dynamic.empty())
   {
    EstimateFeature(frame_2,indices_dynamic);
///Only estimate if they are enough points
    if(frame_1.cloud_input->points.size() > 50 && frame_2.cloud_input->points.size() > 50)
     EstimateCorrespondencePoint(max_motion,sampling_radius,2,index_query,index_match);///Estimate correspondences
    }
   }
  end = SystemClock::now();
  time_diff = end - start;
  feature_time = time_diff.count();


  if(store_results)
  {
   frame_1.raw_input->width = frame_1.raw_input->points.size();
   frame_1.raw_input->height = 1;
   frame_2.raw_input->width = frame_2.raw_input->points.size();
   frame_2.raw_input->height = 1;
   pcl::PCDWriter writer;
   ss.str("");
   ss << output_folder << "a_" << frame_1.frame_id << ".pcd";
   writer.write(ss.str(),*frame_1.raw_input,true);
   ss.str("");
   ss << output_folder << "a_" << frame_2.frame_id << ".pcd";
   writer.write(ss.str(),*frame_2.raw_input,true);

   ss.str("");
   ss << output_folder << "query_a_" << frame_1.frame_id << ".csv";

   ofstream myfile_query(ss.str().c_str());

   ss.str("");
   ss << output_folder << "match_a_" << frame_1.frame_id << ".csv";
   ofstream myfile_match(ss.str().c_str());
   
   for(size_t i = 0; i < index_query.size(); ++i)
   {
    myfile_query << index_query[i] << endl;
    myfile_match << index_match[i] << endl;
   }
   myfile_query.close();
   myfile_match.close();

  }
  PointCloud cloud_static;
  start = SystemClock::now(); 
  if(!index_query.empty())
   EstimateMotion(index_query,index_match,cloud_static);///Estimate the motion
  end = SystemClock::now();
  time_diff = end - start;
  motion_time = time_diff.count();
  end_total = SystemClock::now(); 

  time_diff = end_total - start_total;
  total_time = time_diff.count();

   time_write << feature_time << "," << correspondence_time << "," << motion_time << "," << total_time << "," << frame_1.frame_id << endl;

  
  pcl::toROSMsg(cloud_static,res.cloud_static);////service output, static cloud from potentially dynamic
  frame_2.copy(frame_1);
 }
 
return true;

}


int main(int argc,char **argv)
{
 ros::init(argc, argv, "squirrel_dynamic_filter_service");
 DynamicFilter filter;
	while(ros::ok())
	{
		ros::spinOnce();
 }

}