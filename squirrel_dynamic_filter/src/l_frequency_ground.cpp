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
#include "edge_unary.h"
#include "edge.h"
#include <tf/transform_broadcaster.h>

////used for publishing the points corresponding to the extracted
//ground. Required for clearing the octomap

using namespace std;
using namespace Eigen;
class tfPointCloud
{
	private:

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    ros::Subscriber cloud_sub;
    tf::TransformListener tf_;
    ros::Publisher  publisher;
    tf::MessageFilter <sensor_msgs::PointCloud2> * tf_filter_;
    ros::NodeHandle n_;

    ros::Publisher  pub;   ///publisher for filtrered cloud
    Matrix4f sensor_base_link_trans;
    pcl::VoxelGrid<Point> sor;
    float down_sampling_radius;
    string  input_folder;
		ofstream write_odom;
		pcl::PCDWriter writer;
		tf::StampedTransform old_transform;
    ros::Time start_time;
    squirrel_dynamic_filter_msgs::CloudMsg cloud_msg;
    bool is_verbose;

    sensor_msgs::PointCloud2 ground_msg;
    tf::TransformBroadcaster br;
    tf::Transform transform_map_base_link;

	public:
		tfPointCloud():tf_()
		{
      n_.getParam("DownSamplingRadius",down_sampling_radius);
      n_.getParam("InputFolder",input_folder);
      n_.getParam("Verbose",is_verbose);
      pub = n_.advertise<sensor_msgs::PointCloud2>("/kinect/depth/static_final/",30);//Publising the filtered pointcloud
      cloud_sub_.subscribe(n_, "/kinect/depth/points/",1);///Subscriber
      //cloud_sub = n_.subscribe("/kinect/depth/points/",1,&tfPointCloud::msgCallback, this);///Subscriber
      tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (cloud_sub_, tf_, "base_link", 1);///Filter to synchronize
      tf_filter_->registerCallback(boost::bind(&tfPointCloud::msgCallback, this, _1) );
  //    publisher = n_.advertise<squirrel_dynamic_filter_msgs::CloudMsg>("/squirrel/cloud_msg",100);///publisher
      cloud_msg.odometry.resize(7);
    }
    void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& sensor_msg)
    {

   //Waiting for the pose of the robot
      tf::StampedTransform transform;
      tf_.waitForTransform("/map", "/base_link",sensor_msg->header.stamp, ros::Duration(2.0));
      try
      {
        tf_.lookupTransform("/map", "/base_link",sensor_msg->header.stamp , transform);
      }
      catch (tf::TransformException ex){
      ROS_ERROR("%s,%s",ros::this_node::getName().c_str(),ex.what());
      ros::Duration(1.0).sleep();
      }
 ///Downampling the cloud
      std::stringstream ss;
      ss.str("");
      ss << input_folder << "/params/sensor_to_base_link.csv";

      //cerr << ss.str() << endl;

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
            string line_new=line.substr(found+1);
            line=line_new;
          }
        }
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
      /*cerr << sensor_base_link_trans << endl;*/

      //getchar();


      Matrix4f trans_inverse = sensor_base_link_trans.inverse();


      PointCloud::Ptr cloud(new PointCloud);
      PointCloud::Ptr cloud_sampled(new PointCloud);
      pcl::fromROSMsg(*sensor_msg,*cloud);
      sor.setInputCloud (cloud);
      sor.setLeafSize (0.025,0.025,0.025);
      sor.filter (*cloud_sampled);
      cloud_sampled->width = cloud_sampled->points.size();
      cloud_sampled->height = 1;

         //cerr << sensor_base_link_trans << endl;
       // load_transform = true;
    /*  PointCloud cloud;*/
      /*pcl::fromROSMsg(cloud_msg,cloud);*/
      PointCloud cloud_nan;
      PointCloud cloud_trans;
      for (auto &point:cloud_sampled->points)
      {
        if(pcl_isfinite(point.x))
          cloud_nan.points.push_back(point);
      }

    ///removes infinite points
 ///transforming the scan to base_link. z is now upward and x goes forward. All
 //the ground points and far away points are assumned to be static

      PointCloud cloud_processed;
      pcl::transformPointCloud(cloud_nan,cloud_processed,sensor_base_link_trans);
      //pcl::transformPointCloud(*cloud,cloud_processed,sensor_base_link_trans);
      std::vector <bool> is_ground(cloud_processed.points.size(),false);
      std::vector <int> indices;
      std::vector <int> indices_ground;
      std::vector<int>static_indices;
      std::vector<int>dynamic_indices;

//#pragma omp parallel for
      for(size_t i = 0; i < cloud_processed.points.size();++i)
      {
        Eigen::Vector3f point_eigen = cloud_processed.points[i].getVector3fMap();
        if(cloud_processed.points[i].z < 0.05)
        //if(cloud_processed->points[i].z < 0.05)
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

      PointCloud cloud_ground;

      pcl::copyPointCloud(cloud_processed,static_indices,cloud_ground);

      PointCloud cloud_processed_inverse;
      pcl::transformPointCloud(cloud_ground,cloud_processed_inverse,trans_inverse);
      cloud_ground.width = cloud_ground.points.size();
      cloud_ground.height = 1;

      cloud_nan.width = cloud_nan.points.size();
      cloud_nan.height = 1;


      cloud_processed.width = cloud_processed.points.size();
      cloud_processed.height = 1;
      cloud_processed_inverse.width = cloud_processed_inverse.points.size();
      cloud_processed_inverse.height = 1;


      cloud->width = cloud->points.size();
      cloud->height = 1;



      //std::cerr << cloud_ground.width << endl;

      pcl::toROSMsg(cloud_processed_inverse ,ground_msg);

      br.sendTransform(tf::StampedTransform(transform,sensor_msg->header.stamp, "map", "base_link_static_final"));

//      ground_msg.header.frame_id = "base_link";
      ground_msg.header.frame_id = sensor_msg->header.frame_id;
      ground_msg.header.stamp = sensor_msg->header.stamp;


      pub.publish(ground_msg);
      ros::Duration(0.3).sleep();


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
