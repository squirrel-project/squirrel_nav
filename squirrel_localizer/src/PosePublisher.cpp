// PosePublisher.cpp --- 
// 
// Filename: PosePublisher.cpp
// Description: Publisher for estimated pose
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 12:13:44 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro
//   ROS Indigo
// 

// Code:

#include <ros/ros.h>

#include <ros/time.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

geometry_msgs::PoseWithCovarianceStamped g_msg;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  g_msg = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_pose_publisher");

  ros::NodeHandle n; 
  ros::NodeHandle private_nh_("~"); // private note to parameters

  // Load scan topic names if avaible
  std::string pose_topic;
  std::string from_frame_ID;
  std::string to_frame_ID;

  private_nh_.param("pose_topic", pose_topic, (std::string) "ais_odom_pose");
  private_nh_.param("from_id", from_frame_ID, (std::string) "map");
  private_nh_.param("to_id", to_frame_ID, (std::string) "odom");

  ros::Subscriber sub = n.subscribe(pose_topic, 1, poseCallback);

  g_msg.header.seq = 0;

  tf::TransformBroadcaster br;
  tf::Transform transform;
      tf::Pose pos_tf;

  ros::Rate loop_rate(100);
  while(ros::ok()){
    ros::spinOnce();

    if(g_msg.header.seq != 0){     
      tf::poseMsgToTF(g_msg.pose.pose, pos_tf);   	
      transform.setOrigin(pos_tf.getOrigin());
      transform.setRotation(pos_tf.getRotation());
      br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), from_frame_ID, to_frame_ID));
    }
    loop_rate.sleep();

  }

  return 0;
}

// 
// PosePublisher.cpp ends here
