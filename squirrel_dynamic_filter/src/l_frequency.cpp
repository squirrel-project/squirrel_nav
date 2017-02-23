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

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include "squirrel_dynamic_filter_msgs/CloudMsg.h"
#include "datatypes_squirrel.h"

///Receiving the kinect data and sending the data to the dynamic_filter. This is
//required to compensate for the speed of the code. It publishes the cloud and
//the corresponding pose of the robot

using namespace std;
class tfPointCloud
{
	private:
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    tf::TransformListener tf_;
    ros::Publisher  publisher;
    tf::MessageFilter <sensor_msgs::PointCloud2> * tf_filter_;
    ros::NodeHandle n_;

    pcl::VoxelGrid<Point> sor;
    float down_sampling_radius;
		ofstream write_odom;
		pcl::PCDWriter writer;
		tf::StampedTransform old_transform;
    ros::Time start_time;
    squirrel_dynamic_filter_msgs::CloudMsg cloud_msg;
    bool is_verbose;
	public:
		tfPointCloud():tf_()
		{
      cloud_sub_.subscribe(n_, "/kinect/depth/points/",200);///Subscriber
			tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (cloud_sub_, tf_, "base_link", 1);///Filter to synchronize
      tf_filter_->registerCallback(boost::bind(&tfPointCloud::msgCallback, this, _1) );
      publisher = n_.advertise<squirrel_dynamic_filter_msgs::CloudMsg>("/squirrel/cloud_msg",100);///publisher
      cloud_msg.odometry.resize(7);
      n_.getParam("DownSamplingRadius",down_sampling_radius);
      n_.getParam("Verbose",is_verbose);
    }
    void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& sensor_msg)
    {

   ////Waiting for the pose of the robot
      tf::StampedTransform transform;
      tf_.waitForTransform("/map", "/base_link",sensor_msg->header.stamp, ros::Duration(5.0));
      try
      {
        tf_.lookupTransform("/map", "/base_link",sensor_msg->header.stamp , transform);
      }
      catch (tf::TransformException ex){
      ROS_ERROR("%s,%s",ros::this_node::getName().c_str(),ex.what());
      ros::Duration(1.0).sleep();
      }
 ///Downampling the cloud

      PointCloud::Ptr cloud(new PointCloud);
      PointCloud::Ptr cloud_sampled(new PointCloud);
      pcl::fromROSMsg(*sensor_msg,*cloud);
      sor.setInputCloud (cloud);
      sor.setLeafSize (down_sampling_radius,down_sampling_radius,down_sampling_radius);
      sor.filter (*cloud_sampled);
      cloud_sampled->width = cloud_sampled->points.size();
      cloud_sampled->height = 1;



  ///Publising the cloud and odometry
      pcl::toROSMsg(*cloud_sampled, cloud_msg.cloud_msg);


      cloud_msg.cloud_msg.header.stamp = sensor_msg->header.stamp;
      tf::Vector3 pos = transform.getOrigin();
      tf::Quaternion rot = transform.getRotation();
      cloud_msg.odometry[0] = pos[0];
      cloud_msg.odometry[1] = pos[1];
      cloud_msg.odometry[2] = pos[2];
      cloud_msg.odometry[3] = rot[0];
      cloud_msg.odometry[4] = rot[1];
      cloud_msg.odometry[5] = rot[2];
      cloud_msg.odometry[6] = rot[3];
      publisher.publish(cloud_msg);
    }

};


int main(int argc, char ** argv)
{

  ros::init(argc, argv, "squirrel_dynamic_filter_l_frequency");
	tfPointCloud cloud;
	while(ros::ok())
	{
		ros::spinOnce();
  }
}
