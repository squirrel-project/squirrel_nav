#include <ros/ros.h>
#include <ros/package.h>
#include "squirrel_tracker/squirrel_tracker.h"

int main(int argc, char **argv)
{
//  ROS_INFO("Initialize ROS....\r\n");
  ros::init(argc, argv, "nite_tracker");
  if (!ros::isInitialized())
    return 1;
//  ROS_INFO("Done...\r\n");
  ros::NodeHandle pnh("~");
  SquirrelTracker listener(pnh);
  listener.runSquirrelTracker();
  ros::spin();

  return 0;

}
