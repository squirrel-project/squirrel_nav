// LocalizerApp.h --- 
// 
// Filename: LocalizerApp.h
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:11:01 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//    ROS Hydro
//    ROS Indigo
// 

// Code:

#ifndef SQUIRREL_LOCALIZER_LOCALIZERAPP_H_
#define SQUIRREL_LOCALIZER_LOCALIZERAPP_H_

#include "squirrel_localizer/Localization.h"

#include "squirrel_localizer/RobotLaser.h"
#include "squirrel_localizer/ScanMatcher.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <ros/package.h>

#include <laser_geometry/laser_geometry.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_srvs/Empty.h>

/* #include "squirrel_localizer_msgs/PreciseLocation.h" */
#include "squirrel_localizer_msgs/RefScan.h"

#include <iostream>
#include <sstream>
#include <cstdio>

#include <signal.h>

#include <boost/shared_ptr.hpp>

using namespace std;
using namespace AISNavigation;
namespace aisn = AISNavigation;

bool running = true;
unsigned int breakCounter = 0;

class Ais_localizer_node {
public:
  Ais_localizer_node();
  ~Ais_localizer_node();

  void  entryRunning();
  void  exitRunning();
  void localize();

private:
  ros::NodeHandle nh_;

  bool verbose_;

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
  //void odomCallback(const nav_msgs::Odometry::ConstPtr& odom

  std::vector<Vector3f> local_endpoints_from_laser(const RobotLaser& laser);
  std::vector<Vector3f> endpoints_from_laser(const RobotLaser& laser);

  bool localize_globalCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);

  bool scan_matcher_on_Callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool scan_matcher_off_Callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool updateParameterCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);


  void startLocalization();
  void stopLocalization();

  bool use_last_pose_2_init_;

  //void writeScanToFile(LDP scan, std::string filename);

  // Publisher and Subscriber
  ros::Subscriber scan_front_sub_;
  ros::Subscriber scan_rear_sub_;
  ros::Subscriber initialpose_sub_;
  ros::Publisher ais_particlecloud_pub_;
  ros::Publisher ais_pose_pub_;
  ros::Publisher ais_odom_pose_pub_;
  ros::Publisher ais_pointcloud2_pub_;

  ros::ServiceClient get_map_srv_client_;
  ros::ServiceServer localize_global_srv_ser_;
  ros::ServiceServer take_ref_scan_srv_ser_;
  ros::ServiceServer localize_with_icp_srv_ser_;
  ros::ServiceServer scan_matcher_on_srv_ser_;
  ros::ServiceServer scan_matcher_off_srv_ser_;
  ros::ServiceServer update_param_srv_ser_;
  ros::ServiceServer wirte_out_references_srv_ser_;

  //ros localizer
  double _dMapThreshold;
  double _dMapMinOccupancy;
  bool _dMapMarkUnknown;

  Localization localizer_;
  bool localizer_on_;
  double sm_filter_;
  
  /** Get PARAMS from Param Deamon */
  LocalizerParameters loc_param_;
  LaserSensorParameters laser_param_;
  MotionModelParameters motion_param_;

  /** Load map */
  AISNavigation::FloatMap *map;
  AISNavigation::FloatMap *d_map; //for scan matcher

  /** Draw / publish  stuff */
  Transformation3 endPointsTrans;
  bool stateChanged;
  bool needReDraw;
  bool publish;

  /** Localizer state variables */
  Transformation3 mean;
  Matrix6 covariance;
  bool bounded;
  bool localized;

  /** Main loop */
  RobotLaser robotLaser;

  // make use of both scanners
  bool use_second_laser_;
  bool scan_front_new_;
  bool scan_back_new_;
  sensor_msgs::LaserScan scan_front_;
  sensor_msgs::LaserScan scan_back_;
  std::string front_laser_frame_ID_;
  std::string back_laser_frame_ID_;
  std::string base_link_frame_ID_;
  std::string odom_frame_ID_;
  std::string map_frame_ID_;

  bool init_map_;

  //---------------------------------------------------------------------------
  // scan matcher to distance map
  bool scanMatcherOn;
  ScanMatcher scanMatcher;
  double delta_sm_x, delta_sm_y, delta_sm_theta;

  //---------------------------------------------------------------------------
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster *br_;
  //---------------------------------------------------------------------------

};

#endif  /* SQUIRREL_LOCALIZER_LOCALIZERAPP_H_ */

//
// LocalizerApp.h ends here
