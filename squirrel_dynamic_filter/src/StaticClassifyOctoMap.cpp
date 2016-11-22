#include "ros/ros.h"
#include "datatypes_squirrel.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "squirrel_dynamic_filter_msgs/ClassifyStaticSrv.h"
#include "edge_unary.h"
#include "edge.h"
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace octomap;
using namespace std;
using namespace Eigen;
using namespace g2o;


class ClassifyStatic
{
  private:
    ros::NodeHandle n;
    ros::ServiceServer service;
    ros::Publisher pub_static,pub_dynamic;
    tf::TransformBroadcaster br;
    tf::Transform transform_map_base_link;
    OcTree *tree;
  public:
    string map_filename;
    void load_map()
    {
      tree = new OcTree(map_filename);
    }

    ClassifyStatic()
    {
      service = n.advertiseService("classify_static", &ClassifyStatic::ClassifyStaticCallback,this);
      pub_static = n.advertise<sensor_msgs::PointCloud2>("/kinect/depth/static",10);
      pub_dynamic = n.advertise<sensor_msgs::PointCloud2>("/kinect/depth/dynamic",10);
    }
    bool ClassifyStaticCallback(squirrel_dynamic_filter_msgs::ClassifyStaticSrv::Request &req,squirrel_dynamic_filter_msgs::ClassifyStaticSrv::Response &res)
    {

      Vector7d odometry;

      odometry[0] = req.odometry[0];
      odometry[1] = req.odometry[1];
      odometry[2] = req.odometry[2];
      odometry[3] = req.odometry[3];
      odometry[4] = req.odometry[4];
      odometry[5] = req.odometry[5];
      odometry[6] = req.odometry[6];
      Isometry3D sensor_to_base_link_trans = g2o::internal::fromVectorQT(odometry);
      Matrix4f sensor_base_link_trans;
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
      PointCloud cloud_input_raw;
      PointCloud::Ptr cloud_input(new PointCloud);


  //OcTree *tree = new OcTree(map_filename);

      pcl::fromROSMsg(req.cloud,cloud_input_raw);

      pcl::transformPointCloud(cloud_input_raw,*cloud_input,sensor_base_link_trans);///Transforming cloud in base_link frame

      PointCloud::Ptr static_cloud(new PointCloud);
      PointCloud::Ptr dynamic_cloud(new PointCloud);

      pcl::KdTreeFLANN <Point> kdtree;

      kdtree.setInputCloud(cloud_input);






      res.static_points.clear();
      res.unclassified_points.clear();
    ///if a point is in a occupied voxel then its static, otherswise might be new static or dynamic
      int point_index = 0;
      std::vector <bool> is_processed(cloud_input->points.size(),false);
      for(auto &point:cloud_input->points)
      {
        if(is_processed[point_index])
        {
          point_index+=1;
          continue;
        }
        OcTreeNode *node = tree->search(point.x,point.y,point.z);
        if((node) && (tree->isNodeOccupied(node)))
        {
          std::vector <int> pointIdxRadiusSearch;
          std::vector <float> pointRadiusSquaredDistance;
          if(kdtree.radiusSearch(point,0.2,pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0)
          {
            for(auto &index:pointIdxRadiusSearch)
            {
              res.static_points.push_back(index);///static
              is_processed[index] = true;
            }

          }

        }
        else
          res.unclassified_points.push_back(point_index);//new static or dynamic

        point_index += 1;
      }

      transform_map_base_link.setOrigin(tf::Vector3(odometry[0],odometry[1],odometry[2]));
      tf::Quaternion q(odometry[3],odometry[4],odometry[5],odometry[6]);
      transform_map_base_link.setRotation(q);

      br.sendTransform(tf::StampedTransform(transform_map_base_link, ros::Time::now(), "map", "base_link_static"));//publish the tf corresponding to points

      pcl::copyPointCloud(cloud_input_raw,res.static_points,*static_cloud);//points corresponding to map(static)

      pcl::copyPointCloud(cloud_input_raw,res.unclassified_points,*dynamic_cloud);

    // pcl::transformPointCloud(*static_cloud_map,*static_cloud,sensor_base_link_trans);///Transforming cloud in base_link frame

      sensor_msgs::PointCloud2 cloud_msg;
      sensor_msgs::PointCloud2 cloud_msg_2;


      static_cloud->width = static_cloud->points.size();
      static_cloud->height = 1;
      dynamic_cloud->width = dynamic_cloud->points.size();
      dynamic_cloud->height = 1;



      pcl::toROSMsg(*dynamic_cloud,cloud_msg_2);
      cloud_msg_2.header.frame_id = "base_link_static";
      pcl::toROSMsg(*static_cloud,cloud_msg);
      cloud_msg.header.frame_id = "base_link_static";
      pub_static.publish(cloud_msg);
      pub_dynamic.publish(cloud_msg_2);

      return true;

    }

};

int main(int argc,char **argv)
{
    ros::init(argc, argv, "static_classify");
    ClassifyStatic classify;
    classify.map_filename = argv[1];
    classify.load_map();
    while(ros::ok())
      ros::spinOnce();
}


