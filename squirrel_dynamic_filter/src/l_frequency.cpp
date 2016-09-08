#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include "squirrel_dynamic_filter_msgs/CloudMsg.h"
#include "datatypes_squirrel.h"
using namespace std;
class tfPointCloud
{
	private:

 
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;	           
  tf::TransformListener tf_;
  ros::Publisher  pub;   
  ros::Publisher  pub2;   
  tf::MessageFilter <sensor_msgs::PointCloud2> * tf_filter_;
  ros::NodeHandle n_;

  pcl::VoxelGrid<Point> sor;
  float down_sampling_radius;
  //ros::Subscriber cloud_sub_1 = n_.subscribe("/kinect/depth/points", 1000,&tfPointCloud::Callback, this);
		int counter;
		ofstream write_odom;
		pcl::PCDWriter writer;
		tf::StampedTransform old_transform;
  ros::Time start_time;
  squirrel_dynamic_filter_msgs::CloudMsg cloud_msg;
	public:
  int msg_id;
		tfPointCloud():tf_(),counter(0),msg_id(0)
		{
   cloud_sub_.subscribe(n_, "/kinect/depth/points/",1);
			tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (cloud_sub_, tf_, "base_link", 1);
//		tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (cloud_sub_, tf_, "kinect_rgb_optical_frame", 1000);
		tf_filter_->registerCallback(boost::bind(&tfPointCloud::msgCallback, this, _1) );
  //pub = n_.advertise<sensor_msgs::PointCloud2>("/kinect/depth/slow",1000);
  pub2 = n_.advertise<squirrel_dynamic_filter_msgs::CloudMsg>("/squirrel/cloud_msg",10);
  cloud_msg.odometry.resize(7);
  n_.getParam("DownSamplingRadius",down_sampling_radius); 
		}
  void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& sensor_msg) 
  {
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
 
   PointCloud::Ptr cloud(new PointCloud);
   PointCloud::Ptr cloud_sampled(new PointCloud);

   pcl::fromROSMsg(*sensor_msg,*cloud);
   sor.setInputCloud (cloud);
   sor.setLeafSize (down_sampling_radius,down_sampling_radius,down_sampling_radius);
   sor.filter (*cloud_sampled);
   cloud_sampled->width = cloud_sampled->points.size();
   cloud_sampled->height = 1;

   pcl::toROSMsg(*cloud_sampled, cloud_msg.cloud_msg);
   tf::Vector3 pos = transform.getOrigin();
   tf::Quaternion rot = transform.getRotation();
   
//   cloud_msg.cloud_msg = *sensor_msg;
   cloud_msg.odometry[0] = pos[0];
   cloud_msg.odometry[1] = pos[1];
   cloud_msg.odometry[2] = pos[2];
   cloud_msg.odometry[3] = rot[0];
   cloud_msg.odometry[4] = rot[1];
   cloud_msg.odometry[5] = rot[2];
   cloud_msg.odometry[6] = rot[3];

  // cout << "inside" << endl;

       

   //pub.publish(*sensor_msg);

   //if(counter - msg_id > 1)
 //  {
    pub2.publish(cloud_msg);
    msg_id = counter;
 //  }

   
//    start_time = transform.stamp_;
//   old_transform = transform;

  // cout << (sensor_msg->header.stamp-start_time).toSec() << endl;
//   write_odom << pos.x() << "," << pos.y() << "," <<pos.z() << "," << rot.x() << "," << rot.y() << "," << rot.z() << "," << rot.w() << "," << (sensor_msg->header.stamp-start_time).toSec() << "," << (transform.stamp_-start_time).toSec() << endl;
  // ss<< "/export/data/squirrel/ayush_dataset/ga_meeting_27_2/" << counter << ".pcd";
//   writer.write(ss.str(),*cloud,true);
   counter+=1;



  }
  
  
  };


int main(int argc, char ** argv)
{
	
	
	ros::init(argc, argv, "l_frequency");
	tfPointCloud cloud;
//	ros::Rate rate(10.0);
	while(ros::ok())
	{
		ros::spinOnce();
//		rate.sleep();

	}
}
