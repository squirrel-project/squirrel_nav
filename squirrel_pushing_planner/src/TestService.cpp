#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>

#include <tf/tf.h>

#include "squirrel_rgbd_mapping_msgs/GetPushingPlan.h"

geometry_msgs::Pose2D _odom;

int main( int argc, char *argv[] )
{
  const char *node_name = "test_pushing_planner";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  
  squirrel_rgbd_mapping_msgs::GetPushingPlan srv;

  srv.request.start.x = 0.0;
  srv.request.start.y = -1.0;
  srv.request.start.theta = 1.57;
  
  srv.request.goal.x = 2.0;
  srv.request.goal.y = 0.0;
  srv.request.goal.theta = 0.0;

  geometry_msgs::Point32 p1, p2, p3, p4;
  p1.x = 0.22; p1.y = -0.15;
  p2.x = 0.22; p1.y = 0.15;
  p3.x = 0.62; p1.y = -0.15;
  p4.x = 0.62; p1.y = 0.15;

  srv.request.object.points.push_back(p1);
  srv.request.object.points.push_back(p2);
  srv.request.object.points.push_back(p3);
  srv.request.object.points.push_back(p4);
    
  ros::Rate lr(2.0);
  while ( ros::ok() ) {
    if ( ros::service::call("/getPushingPlan", srv) ) {
      if ( srv.response.plan.poses.empty() ) {
        ROS_WARN("%s: Got an empty plan from [/getPushingPlan]", node_name);
      } else {
        ROS_INFO("%s: got a path for pushing from [/getPushingPlan]", node_name);
      }
    } else {
      ROS_ERROR("%s: unable to communicate with service [/getPushingPlan]", node_name);
    }
    ros::spinOnce();
  }
   
  return 0;
}
