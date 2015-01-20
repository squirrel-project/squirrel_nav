#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>

#include <tf/tf.h>

#include "squirrel_rgbd_mapping_msgs/GetPushingPlan.h"

geometry_msgs::Pose2D _odom;

void getOdometry( const nav_msgs::OdometryConstPtr& odom_msg )
{
  _odom.x = odom_msg->pose.pose.position.x;
  _odom.y = odom_msg->pose.pose.position.y;
  _odom.theta = tf::getYaw(odom_msg->pose.pose.orientation);
}

int main( int argc, char *argv[] )
{
  ros::init(argc, argv,"test_pushing_planner");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/odom", 1000, getOdometry);
  ros::Publisher pub = nh.advertise<nav_msgs::Path>("/test_pushing_planner", 1000);
  
  squirrel_rgbd_mapping_msgs::GetPushingPlan srv;

  srv.request.start.x = 0.600;
  srv.request.start.y = 0.396;
  srv.request.start.theta = 0.683;
  
  srv.request.goal.x = 1.77;
  srv.request.goal.y = -0.1;
  srv.request.goal.theta = 3.14;

  geometry_msgs::Point32 p1, p2, p3, p4;

  p1.x = 0.22; p1.y = -0.15;
  p2.x = 0.22; p1.y = 0.15;
  p3.x = 0.62; p1.y = -0.15;
  p4.x = 0.62; p1.y = 0.15;

  srv.request.object.points.push_back(p1);
  srv.request.object.points.push_back(p2);
  srv.request.object.points.push_back(p3);
  srv.request.object.points.push_back(p4);
    
  ros::Rate lr(1.0);
  
  // while ( ros::ok() ) {
    if ( ros::service::call("/getPushingPlan", srv) ) {
      if ( srv.response.plan.poses.empty() ) {
        ROS_WARN("got an empty plan");
      } else {
        srv.response.plan.header.frame_id = "/map";
        ROS_INFO("got a path for pushing");
      }
    } else {
      ROS_ERROR("unable to communicate with /getPushingPlan");
    }

    ros::spin();
   
  return 0;
}
