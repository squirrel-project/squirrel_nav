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

#include <pcl_ros/transforms.h>
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


 public:

 ClassifyStatic()
 {
  service = n.advertiseService("classify_static", &ClassifyStatic::ClassifyStaticCallback,this);
  pub_static = n.advertise<sensor_msgs::PointCloud2>("/kinect/depth/static",100);
  pub_dynamic = n.advertise<sensor_msgs::PointCloud2>("/kinect/depth/dynamic",100);
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
  PointCloud cloud_input,cloud_input_raw;

  pcl::fromROSMsg(req.cloud,cloud_input_raw);

  pcl::transformPointCloud(cloud_input_raw,cloud_input,sensor_base_link_trans);
  
  OcTree *tree = new OcTree(req.tree_filename);
  PointCloud::Ptr static_cloud(new PointCloud);
  PointCloud::Ptr dynamic_cloud(new PointCloud);

  res.static_points.clear();
  res.unclassified_points.clear();

  int point_index = 0;
  for(auto &point:cloud_input.points)
  {
   OcTreeNode *node = tree->search(point.x,point.y,point.z);
   if((node) && (tree->isNodeOccupied(node)))
    res.static_points.push_back(point_index);
   else
    res.unclassified_points.push_back(point_index);


   point_index += 1;
  }


  pcl::copyPointCloud(cloud_input,res.static_points,*static_cloud);
  pcl::copyPointCloud(cloud_input,res.unclassified_points,*dynamic_cloud);


  sensor_msgs::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2 cloud_msg_2;


  static_cloud->width = static_cloud->points.size();
  static_cloud->height = 1;
  dynamic_cloud->width = dynamic_cloud->points.size();
  dynamic_cloud->height = 1;

  pcl::toROSMsg(*dynamic_cloud,cloud_msg_2);
  cloud_msg_2.header.frame_id = "map";

  pcl::toROSMsg(*static_cloud,cloud_msg);
  cloud_msg.header.frame_id = "map";

  pub_static.publish(cloud_msg);
  pub_dynamic.publish(cloud_msg_2);

  return true;

 }





};











int main(int argc,char **argv)
{

	ros::init(argc, argv, "static_classify");
 ClassifyStatic classify;
 while(ros::ok())
  ros::spinOnce();

/*

 string input_tree = argv[1];
 string occupancy_info = argv[2];
 int start_frame = atoi(argv[3]);
 int end_frame = atoi(argv[4]);
 pcl::PCDReader reader;
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
 std::stringstream ss;
 for(size_t i = start_frame; i < end_frame; ++i)
 {
  fprintf(stderr,"%d\n",i);
  PointCloud::Ptr scan_full(new PointCloud);
  PointCloud scan_full_sampled;
  PointCloud::Ptr scan(new PointCloud);
  ss.str("");
  ss << occupancy_info << i << ".pcd";
  reader.read(ss.str(),*scan_full);

  if(scan_full->points.empty())
   continue;

  std::vector <int> static_indices;
  std::vector <int> dynamic_indices;

  pcl::VoxelGrid<Point> sor;
  
  //int start_s=clock();
 
  sor.setInputCloud (scan_full);
  sor.setLeafSize (0.008,0.008,0.008);
  sor.filter (scan_full_sampled);
//  int stop_s=clock();
//  cout << "time: " << (stop_s-start_s)/double(CLOCKS_PER_SEC)*1000 << endl;
 
  remove_ground(scan_full_sampled,static_indices,dynamic_indices);

  pcl::copyPointCloud(scan_full_sampled,static_indices,*scan);
  OcTree *tree = new OcTree(input_tree);
  for(auto &point:scan->points)
  {
   OcTreeNode *node = tree->search(point.x,point.y,point.z);
   if((node) && (tree->isNodeOccupied(node)))
    static_cloud->points.push_back(point);
   else
    dynamic_cloud->points.push_back(point);
  }

  pcl::visualization::PointCloudColorHandlerCustom <Point> single_color(dynamic_cloud, 0, 255, 0);
  viewer->addPointCloud <Point> (static_cloud,"cloud");
  viewer->addPointCloud <Point> (dynamic_cloud,single_color,"cloud1");
  std::clock_t start;
  double duration;
  start = std::clock(); 

  while (( std::clock() - start ) / (double)CLOCKS_PER_SEC <0.001) 
//  while (!viewer->wasStopped ())
//  while(!close_viewer)
  {
   viewer->spinOnce (100);
   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  viewer->removeAllPointClouds ();
  viewer->removeAllShapes ();






 }

 */


}


