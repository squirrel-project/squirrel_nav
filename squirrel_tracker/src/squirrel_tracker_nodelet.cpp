#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <squirrel_tracker/squirrel_tracker_nodelet.h>
//PLUGINLIB_DECLARE_CLASS(squirrel_tracker, squirrel_tracker_nodelet, squirrel_tracker_nodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(SquirrelTrackerNodelet, nodelet::Nodelet)

SquirrelTrackerNodelet::SquirrelTrackerNodelet() : Nodelet(), listener(NULL) {

}

SquirrelTrackerNodelet::~SquirrelTrackerNodelet() {
  if (listener) {
    delete listener;
    listener = NULL;
  }
}

void SquirrelTrackerNodelet::onInit()
{
  if (!listener) {
    listener = new SquirrelTracker(getPrivateNodeHandle());
  }

  NODELET_INFO("Frame: %s", listener->frame_id.c_str());

  listener->runSquirrelTracker();

}
