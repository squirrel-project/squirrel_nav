#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "squirrel_dynamic_filter_msgs/CloudMsg.h"
#include "edge_unary.h"
#include "edge.h"
#include "datatypes_squirrel.h"
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl_ros/transforms.h>

using namespace std;
using namespace Eigen;
using namespace g2o;


class tfPointCloud
{
	private:
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;	           
  tf::TransformListener tf_;
  ros::Publisher  pub;   
  ros::Publisher  pub2;   
  tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;
  ros::NodeHandle n_;
  int counter;
		ofstream write_odom;
		pcl::PCDWriter writer;
		tf::StampedTransform old_transform;
  ros::Time start_time;
  squirrel_dynamic_filter_msgs::CloudMsg cloud_msg;
  string input_msg;
  bool transform_found;
  bool load_transform;
  string input_folder;
  Matrix4f sensor_base_link_trans; 
  sensor_msgs::PointCloud2 filtered_msg;
	public:
  int msg_id;
		tfPointCloud():tf_(),counter(0),msg_id(0)
		{
   filtered_msg.header.frame_id = "base_link";
   load_transform = false;
   n_.getParam("InputMsg",input_msg); 
   n_.getParam("InputFolder",input_folder); 
   cloud_sub_.subscribe(n_,input_msg,1);
			tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_sub_, tf_,"base_link", 1);
   tf_filter_->registerCallback(boost::bind(&tfPointCloud::msgCallback, this, _1) );
   pub = n_.advertise<sensor_msgs::PointCloud2>("/kinect/depth/slow",1000);
   pub2 = n_.advertise<squirrel_dynamic_filter_msgs::CloudMsg>("/squirrel/cloud_msg",1000);
   cloud_msg.odometry.resize(7);
  }
  
  void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& sensor_msg) 
  {
   n_.getParam("TransformFound",transform_found); 
   if(!transform_found)
    return;
   
   tf::StampedTransform transform;
   tf_.waitForTransform("/map", "/base_link",sensor_msg->header.stamp, ros::Duration(5.0));
   try
   { 
    tf_.lookupTransform("/map", "/base_link",sensor_msg->header.stamp , transform);
   }
   catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
   }
   
   PointCloud::Ptr cloud_processed(new PointCloud);
   PointCloud::Ptr static_cloud(new PointCloud);
   std::vector <int> static_indices;
   std::vector <int> dynamic_indices;

   preprocessing(*sensor_msg,cloud_processed,static_indices,dynamic_indices);   

//   fprintf(stderr,"first one%d,%d,%d\n",cloud_processed->points.size(),static_indices.size(),dynamic_indices.size());
   

   
   pcl::copyPointCloud(*cloud_processed,static_indices,*static_cloud);

   fprintf(stderr,"%d,%d,%d\n",static_cloud->points.size(),cloud_processed->points.size(),dynamic_indices.size());
   static_cloud->width = static_cloud->points.size();
   static_cloud->height = 1;

   cloud_processed->width = cloud_processed->points.size();
   cloud_processed->height = 1;
  
   pcl::toROSMsg(*static_cloud,filtered_msg);
   //pcl::toROSMsg(*cloud_processed,filtered_msg);

   filtered_msg.header.frame_id = "/kinect_rgb_optical_frame";
   pub.publish(filtered_msg);
/*
   tf::Vector3 pos = transform.getOrigin();
   tf::Quaternion rot = transform.getRotation();
   cloud_msg.cloud_msg = *sensor_msg;
   cloud_msg.odometry[0] = pos[0];
   cloud_msg.odometry[1] = pos[1];
   cloud_msg.odometry[2] = pos[2];
   cloud_msg.odometry[3] = rot[0];
   cloud_msg.odometry[4] = rot[1];
   cloud_msg.odometry[5] = rot[2];
   cloud_msg.odometry[6] = rot[3];
   pub2.publish(cloud_msg);
   pub.publish(*sensor_msg);
   msg_id = counter;
   counter+=1;
*/
  }  
  
  void preprocessing(const sensor_msgs::PointCloud2 cloud_msg,PointCloud::Ptr &cloud_processed,std::vector<int>&static_indices,std::vector<int>&dynamic_indices)
  {

   if(!load_transform)
   {
    std::stringstream ss;
    ss.str("");
    ss << input_folder << "/params/sensor_to_base_link.csv";

//    fprintf(stderr,"%s\n",ss.str());
    ifstream myfile(ss.str().c_str());
    std::string line;
    Isometry3D sensor_to_base_link_trans;
    if (myfile.is_open())
    {
     Vector7d trans;
     while (getline (myfile,line))
     {
      for(int i = 0 ; i<7;i++)
      {
     
       std::size_t found = line.find(",");
       if(i==0)
        trans[0] = atof(line.substr(0,found).c_str());
       if(i==1)
        trans[1] = atof(line.substr(0,found).c_str());
       if(i==2)
        trans[2] = atof(line.substr(0,found).c_str());
       if(i==3)
        trans[3] = atof(line.substr(0,found).c_str());
       if(i==4)
        trans[4] = atof(line.substr(0,found).c_str());
       if(i==5)
        trans[5] = atof(line.substr(0,found).c_str());
       if(i==6)
        trans[6] = atof(line.substr(0,found).c_str());
     
   //    fprintf(stderr,"%s\n",line.substr(0,found).c_str());
       string line_new=line.substr(found+1);
       line=line_new;
       
      }
      
    
     }
//     fprintf(stderr,"%f,%f,%f,%f\n",trans[0],trans[1],trans[2],trans[3]);
 //    getchar();
     sensor_to_base_link_trans = g2o::internal::fromVectorQT(trans);
    }
    myfile.close();
    sensor_base_link_trans(0,0) = sensor_to_base_link_trans(0,0);
    sensor_base_link_trans(0,1) = sensor_to_base_link_trans(0,1);
    sensor_base_link_trans(0,2) = sensor_to_base_link_trans(0,2);
    sensor_base_link_trans(0,3) = sensor_to_base_link_trans(0,3);
    sensor_base_link_trans(1,0) = sensor_to_base_link_trans(1,0);
    sensor_base_link_trans(1,1) = sensor_to_base_link_trans(1,1);
    sensor_base_link_trans(1,2) = sensor_to_base_link_trans(1,2);
    sensor_base_link_trans(1,3) = sensor_to_base_link_trans(1,3);
    sensor_base_link_trans(2,0) = sensor_to_base_link_trans(2,0);
    sensor_base_link_trans(2,1) = sensor_to_base_link_trans(2,1);
    sensor_base_link_trans(2,2) = sensor_to_base_link_trans(2,2);
    sensor_base_link_trans(2,3) = sensor_to_base_link_trans(2,3);
    sensor_base_link_trans(3,0) = sensor_to_base_link_trans(3,0);
    sensor_base_link_trans(3,1) = sensor_to_base_link_trans(3,1);
    sensor_base_link_trans(3,2) = sensor_to_base_link_trans(3,2);
    sensor_base_link_trans(3,3) = sensor_to_base_link_trans(3,3);

    load_transform = true;
   }
   PointCloud cloud;
   pcl::fromROSMsg(cloud_msg,cloud);
   PointCloud cloud_nan;
   PointCloud cloud_trans;
   for (auto &point:cloud.points)
   {
    if(pcl_isfinite(point.x))
     cloud_nan.points.push_back(point);
   }///removes infinite points
  
   pcl::transformPointCloud(cloud_nan,*cloud_processed,sensor_base_link_trans);
   std::vector <bool> is_ground(cloud_processed->points.size(),false);
   std::vector <int> indices;
   std::vector <int> indices_ground;
   #pragma omp parallel for
   for(size_t i = 0; i < cloud_processed->points.size();++i)
   {
    if(cloud_processed->points[i].z < 0.02 || cloud_processed->points[i].x > 1.5)
     is_ground[i] = true;
   }
   int count = 0;
   for(auto ground:is_ground)
   {
    if(!ground)
     dynamic_indices.push_back(count);
    else
     static_indices.push_back(count);

    count+=1;
   }

  
//   pcl::copyPointCloud(cloud_trans,indices,*cloud_ground);
/*
   frame.raw_input->points.clear();
   sor.setInputCloud (cloud_ground);
   sor.setLeafSize (down_sampling_radius,down_sampling_radius,down_sampling_radius);
   sor.filter (*frame.raw_input);
   frame.raw_input->width = frame.raw_input->points.size();
   frame.raw_input->height = 1;


   PointCloud::Ptr ground_full(new PointCloud);

   pcl::copyPointCloud(cloud_trans,indices_ground,*ground_full);

   sor.setInputCloud (ground_full);
   sor.setLeafSize (down_sampling_radius,down_sampling_radius,down_sampling_radius);
   sor.filter (*frame.ground);
*/

  }

};


int main(int argc, char ** argv)
{
	
	
	ros::init(argc, argv, "l_frequency");
	tfPointCloud cloud;
	while(ros::ok())
	{
		ros::spinOnce();

	}
}
