#include "squirrel_tracker/squirrel_tracker.h"

SquirrelTracker::SquirrelTracker(ros::NodeHandle& pnh_) :
    pcl_msg(new pcl::PointCloud<pcl::PointXYZ>)
{
  pnh_.param<std::string>("camera_frame_id", frame_id, "camera_depth_frame");
  ostatus = openni::STATUS_OK;
  nstatus = nite::STATUS_OK;
  pubPC = pnh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("filtered_cloud", 1);
  pubPS = pnh_.advertise<geometry_msgs::PoseStamped>("pointing_pose", 10);
  pubPSA = pnh_.advertise<geometry_msgs::PoseArray>("detected_pose", 10);
  pubState = pnh_.advertise<squirrel_tracker_msgs::State>("tracking_state", 10);
  wavingUserID = 0;
  ISWAVEDETECTED = false;
  SWITCHSKELTRACKON = true;
  ISWAVEUSERVISBLE = false;
//  pcl_msg->header.frame_id = frame_id;
//////////////////////////////////////////////////
//  detectedPoseArray.header.frame_id = frame_id;
//  pointingPoint.header.frame_id = frame_id;
////////////////////////////////////////////////
  pcl_msg->height = 1;
  change_frame.setOrigin(tf::Vector3(0, 0, 0));
  //rotate camera frame to robot frame
  tf::Quaternion frame_rotation;
  frame_rotation.setEuler(1.5708, 0, 1.5708);
  change_frame.setRotation(frame_rotation);
  publishedState.state = squirrel_tracker_msgs::State::NO_USER;
  beginUnvis = 0;
  durationUnvis = 3;
  this->initUserTracker();
}

/////////////////////////////////////////////////////////////////////
SquirrelTracker::~SquirrelTracker()
{
  printf("Destructor call!!!");
  shutdownNite();
  shutdownOpenNI();
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::initOpenNI()
{
//  ROS_INFO("Initialize OpenNI....\r\n");
  ostatus = openni::OpenNI::initialize();
  if (ostatus != openni::STATUS_OK)
  {
    ROS_FATAL("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
//-----------------------------------------------------------------
//  ROS_INFO("Opening first device ...\r\n");
  ostatus = device.open(openni::ANY_DEVICE);
  if (ostatus != openni::STATUS_OK)
  {
    ROS_FATAL("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
//  ROS_INFO("%s Opened, Completed.\r\n", device.getDeviceInfo().getName());
//------------------------------------------------------------------
//  ROS_INFO("Create DepthSensor ...\r\n");
  ostatus = depthSensor.create(device, openni::SENSOR_DEPTH);
  if (ostatus != openni::STATUS_OK)
  {
    ROS_FATAL("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
//------------------------------------------------------------------
//  ROS_INFO("DepthSensor start ...\r\n");
  ostatus = depthSensor.start();
  if (ostatus != openni::STATUS_OK)
  {
    ROS_FATAL("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::initNITE()
{
//  ROS_INFO("Initialize NiTE....\r\n");
  nstatus = nite::NiTE::initialize();
  if (nstatus != nite::STATUS_OK)
  {
    ROS_FATAL("Could not initialize Nite. \n %s\r\n", openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
  nstatus = uTracker.create();
  if (nstatus != nite::STATUS_OK)
  {
    ROS_FATAL("Could not create UserTracker. \n %s\r\n", openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
  nstatus = hTracker.create();
  if (nstatus != nite::STATUS_OK)
  {
    ROS_FATAL("Could not create HandTracker. \n %s\r\n", openni::OpenNI::getExtendedError());
    ros::shutdown();
    return;
  }
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::initUserTracker()
{
  initOpenNI();
  initNITE();
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::shutdownOpenNI()
{
//  ROS_INFO("Close Device and Shutdown OpenNI ...\r\n");
  device.close();
  openni::OpenNI::shutdown();
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::shutdownNite()
{
//  ROS_INFO("Destroy UserTracker and Shutdown Nite ...\r\n");
  uTracker.destroy();
  nite::NiTE::shutdown();
}

////////////////////////////////////////////////////////////////////
void SquirrelTracker::setFrameIDParameter()
{
  pcl_msg->header.frame_id = frame_id;
  detectedPoseArray.header.frame_id = frame_id;
  pointingPoint.header.frame_id = frame_id;
}
////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishTransformOfPoint(const nite::UserId& userID, const nite::Point3f& point,
                                              const std::string& frame_id, const std::string& child_frame_id,
                                              const ros::Time& timestamp)
{
  double x = point.x / 1000;
  double y = point.y / 1000;
  double z = point.z / 1000;

  float qx = 0;
  float qy = 0;
  float qz = 0;
  float qw = 1;

  transform.setOrigin(tf::Vector3(-x, y, z));
  transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

  transform = change_frame * transform;
  char child_frame_no[128];
  std::snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), (int)(userID));
  tfBraodcaster.sendTransform(tf::StampedTransform(transform, timestamp, frame_id, child_frame_no));
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishTransformOfJoint(const nite::UserId& userID, const nite::SkeletonJoint& joint,
                                              const std::string& frame_id, const std::string& child_frame_id,
                                              const ros::Time& timestamp)
{
  double x = joint.getPosition().x / 1000;
  double y = joint.getPosition().y / 1000;
  double z = joint.getPosition().z / 1000;

  float qx = joint.getOrientation().x;
  float qy = joint.getOrientation().y;
  float qz = joint.getOrientation().z;
  float qw = joint.getOrientation().w;

  if (qx == qy == qx == qw == 0.0)
  {
    qx = 0;
    qy = 0;
    qz = 0;
    qw = 1;
  }

  transform.setOrigin(tf::Vector3(-x, y, z));
  transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

  transform = change_frame * transform;
  char child_frame_no[128];
  std::snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), (int)(userID));
  tfBraodcaster.sendTransform(tf::StampedTransform(transform, timestamp, frame_id, child_frame_no));
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishPoseStampedArrayOfUsers(const nite::Point3f& point, const ros::Time& timestamp)
{
  detectedPose.orientation.x = 0;
  detectedPose.orientation.y = 0;
  detectedPose.orientation.z = 0;
  detectedPose.orientation.w = 1;

  detectedPose.position.x = point.z / 1000;
  detectedPose.position.y = -point.x / 1000;
  detectedPose.position.z = point.y / 1000;
  detectedPoseArray.header.stamp = timestamp;
  detectedPoseArray.poses.push_back(detectedPose);
  pubPSA.publish(detectedPoseArray);
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishPointingPoseStamped(const nite::Point3f& point, const ros::Time& timestamp)
{
  pointingPoint.pose.orientation.x = 0;
  pointingPoint.pose.orientation.y = 0;
  pointingPoint.pose.orientation.z = 0;
  pointingPoint.pose.orientation.w = 1;

  pointingPoint.pose.position.x = point.z / 1000;
  pointingPoint.pose.position.y = -point.x / 1000;
  pointingPoint.pose.position.z = point.y / 1000;
  pointingPoint.header.stamp = timestamp;

  pubPS.publish(pointingPoint);
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::convertDepthToFilteredPointcloud()
{
  userDepthFrame = userFrame.getDepthFrame();
  openni::DepthPixel zeroPixel = 0;
  int countZeroPixel = 0;
  openni::DepthPixel* depthCell;
  unsigned int nmbrRow = userDepthFrame.getWidth();
  float dX, dY, dZ;

  for (unsigned int y = 0; y < userDepthFrame.getHeight(); y++)
  {
    depthCell = (openni::DepthPixel*)((char*)userDepthFrame.getData() + (y * userDepthFrame.getStrideInBytes()));
    nite::UserId* userPixel = (nite::UserId*)((char*)usersMap.getPixels() + (y * usersMap.getStride()));
    for (unsigned int x = 0; x < userDepthFrame.getWidth(); ++x, ++depthCell, ++userPixel)
    {
      if (*userPixel != 0 || *depthCell == 0)
      {
        ++countZeroPixel;
      }
      else if (*userPixel == 0 && *depthCell != 0)
      {
        ostatus = openni::CoordinateConverter::convertDepthToWorld(depthSensor, (float)x, (float)y, (float)(*depthCell),
                                                                   &dX, &dY, &dZ);
        if (ostatus != openni::STATUS_OK)
        {
          ROS_ERROR("ERROR: #%d, %s", ostatus, openni::OpenNI::getExtendedError());
          return;
        }
        pcl_msg->points.push_back(pcl::PointXYZ(dZ / 1000, -dX / 1000, dY / 1000));
      }
    }
  }
  pcl_msg->width = (userDepthFrame.getWidth() * userDepthFrame.getHeight()) - countZeroPixel;
  return;
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishSkeleton(const nite::UserId& userID, const nite::Skeleton& userSkeleton,
                                      const std::string& frame_id, const ros::Time& timestamp)
{
  if (nstatus == nite::STATUS_OK && userSkeleton.getState() == nite::SKELETON_TRACKED)
  {
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_HEAD), frame_id, "head", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_NECK), frame_id, "neck", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_TORSO), frame_id, "torso", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER), frame_id, "left_shoulder",
                            timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_ELBOW), frame_id, "left_elbow", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_HAND), frame_id, "left_hand", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_SHOULDER), frame_id, "right_shoulder",
                            timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_ELBOW), frame_id, "right_elbow", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_HAND), frame_id, "right_hand", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_HIP), frame_id, "left_hip", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_KNEE), frame_id, "left_knee", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_LEFT_FOOT), frame_id, "left_foot", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_HIP), frame_id, "right_hip", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_KNEE), frame_id, "right_knee", timestamp);
    publishTransformOfJoint(userID, userSkeleton.getJoint(nite::JOINT_RIGHT_FOOT), frame_id, "right_foot", timestamp);
  }
}

/////////////////////////////////////////////////////////////////////
void SquirrelTracker::publishPtCld2(const ros::Time& timestamp)
{
  convertDepthToFilteredPointcloud();
#if ROS_VERSION_MINIMUM(1, 11, 1)  // indigo
  pcl_conversions::toPCL(timestamp, pcl_msg->header.stamp);
#else //hydro
  pcl_msg->header.stamp = timestamp.toNSec();
#endif
//   only indigo line below
//  pcl_conversions::toPCL(timestamp, pcl_msg->header.stamp);
  pubPC.publish(pcl_msg);
  pcl_msg->points.clear();

}
/////////////////////////////////////////////////////////////////////
bool SquirrelTracker::detectWavingUser(const nite::UserId& userID)
{
  nstatus = hTracker.readFrame(&handFrame);
  if (nstatus != nite::STATUS_OK || !handFrame.isValid())
  {
    return false;
  }
  const nite::Array<nite::GestureData>& gestures = handFrame.getGestures();
  for (int i = 0; i < gestures.getSize(); ++i)
  {
    if (gestures[i].getType() == nite::GESTURE_WAVE && gestures[i].isComplete())
    {
      nite::HandId handId;
      nstatus = hTracker.startHandTracking(gestures[i].getCurrentPosition(), &handId);
    }
  }
  const nite::Array<nite::HandData>& hands = handFrame.getHands();
  for (int i = 0; i < hands.getSize(); ++i)
  {
    if (hands[i].isTracking())
    {
      float posX, posY;
      nstatus = hTracker.convertHandCoordinatesToDepth(hands[i].getPosition().x, hands[i].getPosition().y,
                                                       hands[i].getPosition().z, &posX, &posY);
      if (nstatus == nite::STATUS_OK)
      {
        nite::UserId* userIdPtr = (nite::UserId*)((char*)usersMap.getPixels() + ((int)posY * usersMap.getStride()))
            + (int)posX;
        if (userID == *userIdPtr)
        {
          wavingUserID = userID;
          handFrame.release();
          return true;
        }
      }
    }
  }
  return false;
}

/////////////////////////////////////////////////////////////////////
void SquirrelTracker::onNewFrame(nite::UserTracker& uTracker)
{
  nstatus = uTracker.readFrame(&userFrame);
  ros::Time timestamp = ros::Time::now();
  usersMap = userFrame.getUserMap();
  nite::Plane floor = userFrame.getFloor();
  if (nstatus != nite::STATUS_OK || !userFrame.isValid())
  {
    return;
  }
  publishPtCld2(timestamp);

  const nite::Array<nite::UserData>& users = userFrame.getUsers();

  if (!ISWAVEDETECTED)
  {
    int j = 0;
    for (int i = 0; i < users.getSize(); ++i)
    {
      if (!users[i].isVisible())
      {
        ++j;
      }
    }
    if (j > 0)
      publishedState.state = squirrel_tracker_msgs::State::NO_USER;
  }
  for (int i = 0; i < users.getSize(); ++i)
  {
    nite::UserId userID = users[i].getId();
    if (users[i].isVisible())
    {
      if (publishedState.state == squirrel_tracker_msgs::State::NO_USER)
      {
        publishedState.state = squirrel_tracker_msgs::State::VISIBLE_USER;
      }
      publishPoseStampedArrayOfUsers(users[i].getCenterOfMass(), timestamp);
//      publishTransformOfPoint(userID, users[i].getCenterOfMass(), frame_id, "detected_pose", timestamp);
    }

    if (ISWAVEDETECTED && SWITCHSKELTRACKON)
    {
      uTracker.startSkeletonTracking(wavingUserID);
      SWITCHSKELTRACKON = false;
    }
    if (!ISWAVEDETECTED)
    {
      if (detectWavingUser(userID))
      {
        publishedState.state = squirrel_tracker_msgs::State::WAVE_USER;
        ISWAVEDETECTED = true;
        ISWAVEUSERVISBLE = true;
        hTracker.stopGestureDetection(nite::GESTURE_WAVE);
      }
    }
    if (ISWAVEDETECTED && wavingUserID == userID)
    {
      nite::Skeleton userSkeleton = users[i].getSkeleton();
      if (userSkeleton.getState() == nite::SKELETON_TRACKED)
      {
        if (publishedState.state != squirrel_tracker_msgs::State::POINT_USER)
        {
          publishedState.state = squirrel_tracker_msgs::State::SKEL_TRACK_USER;
        }
//        publishSkeleton(wavingUserID, userSkeleton, frame_id, timestamp);
        if (pDetector.isFloorPoint(userSkeleton, floor, floorPoint))
        {
          ROS_INFO("%f,%f,%f", floorPoint.x, floorPoint.y, floorPoint.z);
          publishedState.state = squirrel_tracker_msgs::State::POINT_USER;
          publishPointingPoseStamped(floorPoint, timestamp);
//          publishTransformOfPoint(wavingUserID, point, frame_id, "PointingPoint", timestamp);

        }
        else
        {
          publishedState.state = squirrel_tracker_msgs::State::SKEL_TRACK_USER;
        }
      }
    }
    if (!users[i].isVisible() && wavingUserID == userID && ISWAVEUSERVISBLE)
    {
      ISWAVEUSERVISBLE = false;
      beginUnvis = timestamp.toSec();

    }
    if (users[i].isVisible() && wavingUserID == userID && ISWAVEDETECTED)
    {
      ISWAVEUSERVISBLE = true;
    }
    if (!ISWAVEUSERVISBLE && wavingUserID == userID && (timestamp.toSec() - beginUnvis) > durationUnvis)
    {
      publishedState.state = squirrel_tracker_msgs::State::NO_USER;
      ISWAVEDETECTED = false;
      hTracker.startGestureDetection(nite::GESTURE_WAVE);
      SWITCHSKELTRACKON = true;
      uTracker.stopSkeletonTracking(wavingUserID);
      wavingUserID = 0;

    }
  }
  /////PoseArrayTest
  pubState.publish(publishedState);
  detectedPoseArray.poses.clear();
  userFrame.release();
}
/////////////////////////////////////////////////////////////////////
void SquirrelTracker::runSquirrelTracker()
{
  setFrameIDParameter();
  hTracker.startGestureDetection(nite::GESTURE_WAVE);
  uTracker.addNewFrameListener(this);
}
