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

  //dynamic_filter_service = n_.advertiseService("dynamic_filter",&DynamicFilter::DynamicFilterSrvCallback,this);

  cloud_sub = n_.subscribe("/squirrel/dynamic_filter_msg", 1000, &DynamicFilter::msgCallback, this);

  static_cloud_pub = n_.advertise<sensor_msgs::PointCloud2>("/kinect/depth/static_final/",10);//Publising the filtered pointcloud
}
void DynamicFilter::msgCallback(const squirrel_dynamic_filter_msgs::DynamicFilterMsg& dynamic_msg)
{
  if(!is_first_frame)
  {

    if(dynamic_msg.frame_id != frame_1.frame_id + 1)
      is_first_frame = true;
  }
  if(is_first_frame)
  {
    frame_1.clear();
    pcl::fromROSMsg(dynamic_msg.cloud,*frame_1.raw_input);
    Vector7d odometry;
    odometry[0] = dynamic_msg.odometry[0];
    odometry[1] = dynamic_msg.odometry[1];
    odometry[2] = dynamic_msg.odometry[2];
    odometry[3] = dynamic_msg.odometry[3];
    odometry[4] = dynamic_msg.odometry[4];
    odometry[5] = dynamic_msg.odometry[5];
    odometry[6] = dynamic_msg.odometry[6];
    frame_1.odometry = g2o::internal::fromVectorQT(odometry);
    frame_1.frame_id = dynamic_msg.frame_id;
    EstimateFeature(frame_1);
    is_first_frame = false;
    if(is_verbose)
      ROS_INFO("first frame %s:%ld,%d",ros::this_node::getName().c_str(),frame_1.raw_input->points.size(),frame_1.frame_id);
  }
  else
  {

    start_total = SystemClock::now();
    frame_2.clear();
    pcl::fromROSMsg(dynamic_msg.cloud,*frame_2.raw_input);
    Vector7d odometry;
    odometry[0] = dynamic_msg.odometry[0];
    odometry[1] = dynamic_msg.odometry[1];
    odometry[2] = dynamic_msg.odometry[2];
    odometry[3] = dynamic_msg.odometry[3];
    odometry[4] = dynamic_msg.odometry[4];
    odometry[5] = dynamic_msg.odometry[5];
    odometry[6] = dynamic_msg.odometry[6];
    frame_2.odometry = g2o::internal::fromVectorQT(odometry);
    frame_2.frame_id = dynamic_msg.frame_id;
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
   //

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
    sensor_msgs::PointCloud2 cloud_static_msg;

    Eigen::Matrix4f frame_to_map = Eigen::Matrix4f::Identity();

    frame_to_map(0,0) = frame_2.odometry(0,0);
    frame_to_map(0,1) = frame_2.odometry(0,1);
    frame_to_map(0,2) = frame_2.odometry(0,2);
    frame_to_map(0,3) = frame_2.odometry(0,3);

    frame_to_map(1,0) = frame_2.odometry(1,0);
    frame_to_map(1,1) = frame_2.odometry(1,1);
    frame_to_map(1,2) = frame_2.odometry(1,2);
    frame_to_map(1,3) = frame_2.odometry(1,3);

    frame_to_map(2,0) = frame_2.odometry(2,0);
    frame_to_map(2,1) = frame_2.odometry(2,1);
    frame_to_map(2,2) = frame_2.odometry(2,2);
    frame_to_map(2,3) = frame_2.odometry(2,3);

    pcl::toROSMsg(cloud_static,cloud_static_msg);////service output, static cloud from potentially dynamic


    cloud_static_msg.header.frame_id = "base_link_static_final";
    cloud_static_msg.header.stamp= ros::Time::now();
    //cloud_static_msg.header.frame_id = "map";
    transform_map_base_link.setOrigin(tf::Vector3(odometry[0],odometry[1],odometry[2]));
    tf::Quaternion q(odometry[3],odometry[4],odometry[5],odometry[6]);
    transform_map_base_link.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform_map_base_link, ros::Time::now(), "map", "base_link_static_final"));//publish the tf corresponding to points

    static_cloud_pub.publish(cloud_static_msg);
    frame_2.copy(frame_1);
  }

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
