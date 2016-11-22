#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include "squirrel_dynamic_filter_msgs/CloudMsg.h"
#include "squirrel_dynamic_filter_msgs/DynamicFilterSrv.h"
#include "squirrel_dynamic_filter_msgs/DynamicFilterMsg.h"
#include "squirrel_dynamic_filter_msgs/ClassifyStaticSrv.h"
#include "edge_unary.h"
#include "edge.h"
#include "datatypes_squirrel.h"
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;
using namespace g2o;


class tfPointCloud
{
  private:
    ros::Subscriber cloud_sub;//subsriber to messahe from l_frequency
    ros::Publisher  pub;   ///publisher for filtrered cloud

    ros::Publisher dynamic_filter_msg_pub;
    ros::NodeHandle n_;
    int counter;
    squirrel_dynamic_filter_msgs::CloudMsg cloud_msg;
    string input_msg;
    bool transform_found;
    bool load_transform;
    string input_folder;
    string output_folder;
    Matrix4f sensor_base_link_trans;
    sensor_msgs::PointCloud2 filtered_msg;
    sensor_msgs::PointCloud2 full_scene;
    ros::ServiceClient classify_static_client = n_.serviceClient<squirrel_dynamic_filter_msgs::ClassifyStaticSrv>("classify_static");
    squirrel_dynamic_filter_msgs::ClassifyStaticSrv classify_static_srv;
    ros::ServiceClient client = n_.serviceClient<squirrel_dynamic_filter_msgs::DynamicFilterSrv>("dynamic_filter");
    squirrel_dynamic_filter_msgs::DynamicFilterSrv dynamic_srv;
    squirrel_dynamic_filter_msgs::DynamicFilterMsg dynamic_filter_msg;
    std::stringstream ss;
    float down_sampling_radius;
    float static_front_threshold;
    PointCloud previous_cloud;
    PointCloud previous_dynamic;
    bool is_verbose;
    tf::TransformBroadcaster br;
    tf::Transform transform_map_base_link;
    ofstream time_write;
  public:
    tfPointCloud():counter(0)
    {
      filtered_msg.header.frame_id = "base_link";
      load_transform = false;
      n_.getParam("InputMsg",input_msg);
      n_.getParam("InputFolder",input_folder);
      n_.getParam("OutputFolder",output_folder);
      n_.getParam("DownSamplingRadius",down_sampling_radius);
      n_.getParam("StaticFrontThreshold",static_front_threshold);
      n_.getParam("Verbose",is_verbose);
      pub = n_.advertise<sensor_msgs::PointCloud2>("/kinect/depth/static_final/",10);//Publising the filtered pointcloud
      dynamic_filter_msg_pub = n_.advertise<squirrel_dynamic_filter_msgs::DynamicFilterMsg>("/squirrel/dynamic_filter_msg",10);//Publising the filtered pointcloud
      cloud_sub = n_.subscribe("/squirrel/cloud_msg", 500, &tfPointCloud::msgCallback, this);
          ///subscribing to the message sent by l_frequency
      dynamic_srv.request.odometry.resize(7);
      dynamic_filter_msg.odometry.resize(7);
      std::stringstream ss;
      ss.str("");
      ss << output_folder << "time.csv";
      time_write.open(ss.str().c_str());

    }
    void msgCallback(const squirrel_dynamic_filter_msgs::CloudMsg& sensor_msg)
    {
      n_.getParam("TransformFound",transform_found);
      if(!transform_found)
      {
        ROS_INFO_STREAM(ros::this_node::getName() << ":waiting for the transform");
        return;
      }

          ////The input cloud goes through a preprocessing step which involves
          //estimatimg the static points and potentially dynamic points
       PointCloud::Ptr cloud_processed(new PointCloud);
       PointCloud::Ptr static_cloud(new PointCloud);///Static
       PointCloud::Ptr dynamic_cloud(new PointCloud);///dynamic
       PointCloud::Ptr ground(new PointCloud);///ground and far away points
       PointCloud::Ptr not_ground(new PointCloud);///not ground
       std::vector <int> ground_indices;// indices from cloud_processed
       std::vector <int> non_ground_indices;//indices from cloud_processed
       preprocessing(sensor_msg.cloud_msg,cloud_processed,ground_indices,non_ground_indices);//remove ground and far away points

       pcl::copyPointCloud(*cloud_processed,ground_indices,*ground);//ground
       pcl::copyPointCloud(*cloud_processed,non_ground_indices,*not_ground);//non-ground
       not_ground->width = not_ground->points.size();
       not_ground->height = 1;

          /*   dynamic_cloud->width = dynamic_cloud->points.size();*/
             //dynamic_cloud->height = 1;
       pcl::toROSMsg(*not_ground,classify_static_srv.request.cloud);
       classify_static_srv.request.odometry.resize(7);
       classify_static_srv.request.odometry[0] = sensor_msg.odometry[0];
       classify_static_srv.request.odometry[1] = sensor_msg.odometry[1];
       classify_static_srv.request.odometry[2] = sensor_msg.odometry[2];
       classify_static_srv.request.odometry[3] = sensor_msg.odometry[3];
       classify_static_srv.request.odometry[4] = sensor_msg.odometry[4];
       classify_static_srv.request.odometry[5] = sensor_msg.odometry[5];
       classify_static_srv.request.odometry[6] = sensor_msg.odometry[6];
       measure_time start = SystemClock::now();
       if(not_ground->points.size() > 50)
       {
         if(classify_static_client.call(classify_static_srv))
          {
           pcl::copyPointCloud(*not_ground,classify_static_srv.response.static_points,*static_cloud);///to be used for localization
           pcl::copyPointCloud(*not_ground,classify_static_srv.response.unclassified_points,*dynamic_cloud);
         }
       }
       measure_time end = SystemClock::now();
       TimeDiff time_diff = end - start;
       double correspondence_time = time_diff.count();
       time_write << correspondence_time << endl;

       pcl::toROSMsg(*dynamic_cloud,dynamic_filter_msg.cloud);
       dynamic_filter_msg.odometry[0] = sensor_msg.odometry[0];
       dynamic_filter_msg.odometry[1] = sensor_msg.odometry[1];
       dynamic_filter_msg.odometry[2] = sensor_msg.odometry[2];
       dynamic_filter_msg.odometry[3] = sensor_msg.odometry[3];
       dynamic_filter_msg.odometry[4] = sensor_msg.odometry[4];
       dynamic_filter_msg.odometry[5] = sensor_msg.odometry[5];
       dynamic_filter_msg.odometry[6] = sensor_msg.odometry[6];
       dynamic_filter_msg.frame_id = counter;
       if(dynamic_cloud->points.size() > 50)
         dynamic_filter_msg_pub.publish(dynamic_filter_msg);
       counter+=1;

    }
    void preprocessing(const sensor_msgs::PointCloud2 cloud_msg,PointCloud::Ptr &cloud_processed,std::vector<int>&static_indices,std::vector<int>&dynamic_indices)
    {
///if its first frame then load the transform between sensor and base_link
      if(!load_transform)
      {
        std::stringstream ss;
        ss.str("");
        ss << input_folder << "/params/sensor_to_base_link.csv";
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
      }

    ///removes infinite points
 ///transforming the scan to base_link. z is now upward and x goes forward. All
 //the ground points and far away points are assumned to be static
      pcl::transformPointCloud(cloud_nan,*cloud_processed,sensor_base_link_trans);
      std::vector <bool> is_ground(cloud_processed->points.size(),false);
      std::vector <int> indices;
      std::vector <int> indices_ground;
#pragma omp parallel for
      for(size_t i = 0; i < cloud_processed->points.size();++i)
      {
        Eigen::Vector3f point_eigen = cloud_processed->points[i].getVector3fMap();
        if(cloud_processed->points[i].z < 0.05 || point_eigen.lpNorm<2>() > static_front_threshold)
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
    }

};


int main(int argc, char ** argv)
{	ros::init(argc, argv, "squirrel_dynamic_filter_preprocessing");
	tfPointCloud cloud;
	while(ros::ok())
	{
		ros::spinOnce();

	}
}
