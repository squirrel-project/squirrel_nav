// LocalizerApp.cpp --- 
// 
// Filename: LocalizerApp.cpp
// Description: Localization node for Squirrel Project
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 12:12:06 2015 (+0100)
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

#include "squirrel_localizer/LocalizerApp.h"

#include <ros/time.h>

Ais_localizer_node::Ais_localizer_node() :
    verbose_(false),
    use_last_pose_2_init_(true),
    scan_front_new_(false),
    scan_back_new_(false)
{
  ros::NodeHandle private_nh_("~");

  // Load scan topic names if avaible
  std::string scan_front_topic, scan_rear_topic;
  private_nh_.param("scan_front_topic", scan_front_topic, (std::string) "scan_front");
  private_nh_.param("scan_rear_topic", scan_rear_topic, (std::string) "scan_rear");

  // msg subscriber
  scan_front_sub_ = nh_.subscribe < sensor_msgs::LaserScan
                                    > (scan_front_topic, 1, &Ais_localizer_node::scanCallback, this);
  scan_rear_sub_ = nh_.subscribe < sensor_msgs::LaserScan
                                   > (scan_rear_topic, 1, &Ais_localizer_node::scanCallback, this);
  initialpose_sub_ = nh_.subscribe < geometry_msgs::PoseWithCovarianceStamped
                                     > ("initialpose", 1, &Ais_localizer_node::initialPoseCallback, this);

  // msg publisher
  ais_particlecloud_pub_ = nh_.advertise < geometry_msgs::PoseArray > ("particlecloud", 2);
  ais_pose_pub_ = nh_.advertise < geometry_msgs::PoseWithCovarianceStamped > ("squirrel_localizer_pose", 2);
  ais_odom_pose_pub_ = nh_.advertise < geometry_msgs::PoseWithCovarianceStamped > ("squirrel_localizer_odom_pose", 2);
  ais_pointcloud2_pub_ = nh_.advertise < sensor_msgs::PointCloud2 > ("ais_pointcloud2", 2);

  // Services servers
  localize_global_srv_ser_ = nh_.advertiseService("global_localization", &Ais_localizer_node::localize_globalCallback,
                                                  this);

  scan_matcher_on_srv_ser_ = nh_.advertiseService("scan_matcher_on", &Ais_localizer_node::scan_matcher_on_Callback,
                                                  this);
  scan_matcher_off_srv_ser_ = nh_.advertiseService("scan_matcher_off", &Ais_localizer_node::scan_matcher_off_Callback,
                                                   this);
  update_param_srv_ser_ = nh_.advertiseService("localizer_update_parameter",
                                               &Ais_localizer_node::updateParameterCallback, this);


	
  private_nh_.param("localizer_on", localizer_on_, true);
  private_nh_.param("verbose",verbose_,false);
  private_nh_.param("use_last_pose",use_last_pose_2_init_,use_last_pose_2_init_);

  private_nh_.param("use_second_laser", use_second_laser_, true);
  private_nh_.param("front_laser_frame_ID", front_laser_frame_ID_, (std::string) "/laserFront_frame");
  private_nh_.param("rear_laser_frame_ID", back_laser_frame_ID_, (std::string) "/laserRear_frame");
  private_nh_.param("base_link_frame_ID", base_link_frame_ID_, (std::string) "/base_link");
  private_nh_.param("odom_frame_ID", odom_frame_ID_, (std::string) "/odom");
  private_nh_.param("map_frame_ID", map_frame_ID_, (std::string) "/map");

  //---------------------------------------------------------------------------
  // Parameter for particle filter
  private_nh_.param("dMapTheshold", _dMapThreshold, 2.0); //2.0
  _dMapMinOccupancy = 0.65; // 0.5;
  _dMapMarkUnknown = false;

  private_nh_.param("minWeight", loc_param_.minWeight, 0.01);
  private_nh_.param("number_of_particles", loc_param_.particles, 5000);
  private_nh_.param("resample_variance", loc_param_.resample_variance, 0.5);
  loc_param_.use_secondary_laser = 0; //DO NOT USE! It must be '0'! The rosnode takes care of it.
  loc_param_.min_occupancy = _dMapMinOccupancy;
  loc_param_.max_distance = _dMapThreshold;
  loc_param_.mark_unknown = _dMapMarkUnknown;

  private_nh_.param("fullThreshold", laser_param_.fullThreshold, 0.51);
  private_nh_.param("maxLocalizationRange", laser_param_.maxLocalizationRange, 30.0);
  private_nh_.param("maxRange", laser_param_.maxRange, 40.);
  private_nh_.param("usableRange", laser_param_.usableRange, 35.);
  private_nh_.param("observationPointDensity", laser_param_.observationPointDensity, 1.0);
  private_nh_.param("observationSigma", laser_param_.observationSigma, 0.2);
  private_nh_.param("angularUpdate", laser_param_.angularUpdate, 0.2);
  private_nh_.param("convergenceAngle", laser_param_.convergenceAngle, 0.3);
  private_nh_.param("convergenceDistance", laser_param_.convergenceDistance, 0.2);
  private_nh_.param("convergenceRadius", laser_param_.convergenceRadius, 0.5);
  private_nh_.param("disagreementThreshold", laser_param_.disagreementThreshold, 15);
  private_nh_.param("divergenceDistance", laser_param_.divergenceDistance, 0.5);
  private_nh_.param("linearUpdate", laser_param_.linearUpdate, 0.2);
  private_nh_.param("weight", laser_param_.weight, 10.);

  private_nh_.param("motion_param_ff", motion_param_.ff, 0.2);
  private_nh_.param("motion_param_fr", motion_param_.fr, 0.1);
  private_nh_.param("motion_param_fs", motion_param_.fs, 0.1);
  private_nh_.param("motion_param_rr", motion_param_.rr, 0.2);
  private_nh_.param("motion_param_sr", motion_param_.sr, 0.1);
  private_nh_.param("motion_param_ss", motion_param_.ss, 0.1);
  private_nh_.param("motion_param_sth", motion_param_.sth, 0.);
  private_nh_.param("motion_param_sx", motion_param_.sx, 0.);
  private_nh_.param("motion_param_sy", motion_param_.sy, 0.);
  private_nh_.param("motion_param_magnitude", motion_param_.magnitude, 1.);

  localizer_.m_localizer_params = loc_param_;
  localizer_.m_laser_sensor_params = laser_param_;
  localizer_.m_motion_model_params = motion_param_;
  localizer_.reload_parameters();
  localizer_.set_initialized(false);

  // map
  map = new AISNavigation::FloatMap;

  // state variables for MCL
  mean = Transformation3();
  covariance = Matrix6();
  bounded = false;
  localized = false;
  localized = localizer_.has_converged(mean, covariance, bounded);

  //---------------------------------------------------------------------------
  // Scan matcher (scan to map)
  private_nh_.param("use_scan_to_map_matching", scanMatcherOn, false);
  delta_sm_x = delta_sm_y = delta_sm_theta = 0;
  //---------------------------------------------------------------------------


  // Load static map
  init_map_ = false;
  get_map_srv_client_ = nh_.serviceClient < nav_msgs::GetMap > ("static_map");
  nav_msgs::GetMap srv_get_map;
  // cout << "############# Wait for map #############" << endl;
  do {
    get_map_srv_client_.call(srv_get_map);
  } while (srv_get_map.response.map.data.size() == 0);

  init_map_ = false;
  mapCallback(boost::make_shared < nav_msgs::OccupancyGrid > (srv_get_map.response.map));

  ros::Duration(3).sleep(); // wait 3 seconds to buffer tf listener

  br_ = new tf::TransformBroadcaster();

  if (use_last_pose_2_init_) {
    string file_name = "last_pose_localizer_pose.txt";	  
    std::filebuf fb;
    if (NULL != fb.open(file_name.c_str(), std::ios::in)) {
      std::istream in_stream(&fb);
      double x, y, theta;
      in_stream >> x;
      in_stream >> y;
      in_stream >> theta;
      fb.close();

      double sigma = 0.2;
      bool usePoseTheta = true;
      Vector6 vec;
      vec.fill(0.0);
      vec[0] = x;
      vec[1] = y;
      vec[5] = theta;
      localizer_.set_localized(true);
      localizer_.place_robot(Transformation3::fromVector(vec), sigma, usePoseTheta);
    }
  } else {
    double sigma = 0.2;
    bool usePoseTheta = true;
    Vector6 vec;
    vec.fill(0.0);
    localizer_.set_localized(true);
    localizer_.place_robot(Transformation3::fromVector(vec), sigma, usePoseTheta);
  }
}

void Ais_localizer_node::entryRunning() {
  localizer_on_ = true;
}

void Ais_localizer_node::exitRunning() {
  localizer_on_ = false;
}


Ais_localizer_node::~Ais_localizer_node() {

  Transformation3 mean = localizer_.getMean();

  double x = mean._translation.x();
  double y = mean._translation.y();
  double theta = mean.toVector()[5];
  
  string file_name = "last_pose_localizer_pose.txt";
  std::ofstream out_file(file_name.c_str());
  out_file << x << "\n";
  out_file << y << "\n";
  out_file << theta << "\n";
  out_file.close();

  delete map;
}


bool Ais_localizer_node::localize_globalCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

  localizer_.set_localized(false);
  localizer_.start_global();
  
  return true;
}

void Ais_localizer_node::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
  double sigma = 0.2;
  bool usePoseTheta = true;
  Vector6 vec;
  vec.fill(0.0);
  vec[0] = pose->pose.pose.position.x;
  vec[1] = pose->pose.pose.position.y;
  vec[5] = tf::getYaw(pose->pose.pose.orientation);

  localizer_.set_localized(false);
  localizer_.place_robot(Transformation3::fromVector(vec), sigma, usePoseTheta);

}


bool Ais_localizer_node::scan_matcher_on_Callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

  scanMatcherOn = true;
  delta_sm_x = delta_sm_y = delta_sm_theta = 0;

  return true;
}

bool Ais_localizer_node::scan_matcher_off_Callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

  scanMatcherOn = false;
  return true;
}


bool Ais_localizer_node::updateParameterCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

  ros::NodeHandle private_nh_("~");

  private_nh_.param("use_second_laser", use_second_laser_, true);
  private_nh_.param("front_laser_frame_ID", front_laser_frame_ID_, (std::string) "/laserFront_frame");
  private_nh_.param("rear_laser_frame_ID", back_laser_frame_ID_, (std::string) "/laserRear_frame");
  private_nh_.param("base_link_frame_ID", base_link_frame_ID_, (std::string) "/base_link");
  private_nh_.param("odom_frame_ID", odom_frame_ID_, (std::string) "/odom");
  private_nh_.param("map_frame_ID", map_frame_ID_, (std::string) "/map");

  return true;
}

void Ais_localizer_node::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {

  // cout << "############# Got Map #############" << endl;
  if (!init_map_) {
    // cout << "############# Init Map #############" << endl;
    init_map_ = this->map->fromInt8Vector(map->info.width, map->info.height, map->info.origin.position.x,
                                          map->info.origin.position.y, map->info.resolution, map->data);

    localizer_.set_map(this->map);
    d_map = localizer_.get_d_map(); // get distance map from localizer (sensor)

    ScanMatcher scanMatcher(d_map); // Scan matcher: scan to map
    this->scanMatcher = scanMatcher;

    // cerr << "\tMap size: " << this->map->size() << endl;
    //localizer_.start_global();

  }
}

void Ais_localizer_node::localize() {

    
  if (!localizer_on_) {
    return;
  }

  if( !scan_front_new_){
    return;
  }

  if( use_second_laser_ && !scan_back_new_){
    return;
  }

  scan_front_new_ = false;
  scan_back_new_ = false;




  //##############################################################################
  // Wait and get Transforms
  //##############################################################################
  tf::StampedTransform transform_odom2baseLink_frontTime;
  tf::StampedTransform transform_Front_laser;

  tf::StampedTransform transform_odom2baseLink_backTime;
  tf::StampedTransform transform_Back_laser;

  //Wait for transform
  try {
    if (use_second_laser_) {
      tf_listener_.waitForTransform(odom_frame_ID_, base_link_frame_ID_, scan_back_.header.stamp,
                                    ros::Duration(1.0));
      tf_listener_.waitForTransform(base_link_frame_ID_, back_laser_frame_ID_, scan_front_.header.stamp,
                                    ros::Duration(1.0));
    }

    tf_listener_.waitForTransform(odom_frame_ID_, base_link_frame_ID_, scan_front_.header.stamp,
                                  ros::Duration(1.0));

    ros::Time laser_start_time = scan_front_.header.stamp;
    ros::Time laser_end_time = laser_start_time + ros::Duration().fromSec(scan_front_.ranges.size() * scan_front_.time_increment);

    tf_listener_.waitForTransform(base_link_frame_ID_, front_laser_frame_ID_, laser_end_time,
                                  ros::Duration(1.0));


  } catch (tf::TransformException& ex) {
    std::string ns = ros::this_node::getNamespace();
    std::string node_name = ros::this_node::getName();
    ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
    // ROS_ERROR_STREAM(ns << "/" << node_name << ": Not localized in this step because of tf error while waiting for Transform. Return and continue as normal.");
    return;
  }

  // Loop up transform
  if (use_second_laser_) {
    //  listener_odom_tf.lookupTransform(odom_frame_ID_, base_link_frame_ID_,
    //		  scan_Back.header.stamp, transform_odom2baseLink_backTime);

    try {
      tf_listener_.lookupTransform(base_link_frame_ID_, back_laser_frame_ID_, scan_back_.header.stamp,
                                   transform_Back_laser);
    } catch (tf::TransformException& ex) {
      std::string ns = ros::this_node::getNamespace();
      std::string node_name = ros::this_node::getName();
      ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
      // ROS_ERROR_STREAM(
      //     "Not localized in this step because of tf error (base_link to back_laser_frame_ID_). Return and continue as normal.");
      return;
    }

  }

  try {
    tf_listener_.lookupTransform(odom_frame_ID_, base_link_frame_ID_, scan_front_.header.stamp,
                                 transform_odom2baseLink_frontTime);
  } catch (tf::TransformException& ex) {
    std::string ns = ros::this_node::getNamespace();
    std::string node_name = ros::this_node::getName();
    ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
    // ROS_ERROR_STREAM(
    //     "Not localized in this step because of tf error (odom to base_link). Return and continue as normal.");
    return;
  }

  try {
    tf_listener_.lookupTransform(base_link_frame_ID_, front_laser_frame_ID_, scan_front_.header.stamp,
                                 transform_Front_laser);

  } catch (tf::TransformException& ex) {
    std::string ns = ros::this_node::getNamespace();
    std::string node_name = ros::this_node::getName();
    ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
    // ROS_ERROR_STREAM(
    //     "Not localized in this step because of tf error (base_link to front_laser_frame_ID_). Return and continue as normal.");
    return;
  }
  //##############################################################################

  //##############################################################################
  // compute the endpoints in the base_link frame (localized frame)
  //
  // get relevant endpoints in base link frame at the time
  // when the front scan was taken
  //##############################################################################

  robotLaser.m_parameters.maxRange = laser_param_.usableRange;
  robotLaser.m_parameters.angularResolution = scan_front_.angle_increment;
  robotLaser.m_parameters.startAngle = scan_front_.angle_min;
  robotLaser.m_ranges = scan_front_.ranges;

  robotLaser.m_timeStamp = scan_front_.header.stamp.toSec();

  //------------------------------------------------------------------------------
  // create point cloud (only front laser data)

  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud2 cloudFrontScan;
  try {
    projector.transformLaserScanToPointCloud(base_link_frame_ID_, scan_front_, cloudFrontScan, tf_listener_,
                                             laser_param_.usableRange, 8);
  } catch (tf::TransformException& ex) {
    std::string ns = ros::this_node::getNamespace();
    std::string node_name = ros::this_node::getName();
    ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
    // ROS_ERROR_STREAM("Not localized in this step because of tf error (base_link to scan_front_) and point cloud. Return and continue as normal.");
    return;
  }

  ais_pointcloud2_pub_.publish(cloudFrontScan);
  std::vector < Vector3f > endPoints;
  pcl::PointCloud < pcl::PointXYZ > PointCloudXYZ;

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(cloudFrontScan, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, PointCloudXYZ);

  int cloudsize = (PointCloudXYZ.width) * (PointCloudXYZ.height);

  for (int i = 0; i < cloudsize; i++) {
    endPoints.push_back(Vector3f(PointCloudXYZ.points[i].x, PointCloudXYZ.points[i].y, 0.));
  }

  //------------------------------------------------------------------------------
  // Transform back laser data to robot base_link frame
  // (at the time when the front laser scan was taken)
  // back laser offset + offset due to travel
  if (use_second_laser_) {
    sensor_msgs::PointCloud2 cloudBackScan_timeOld, cloudBackScan;
    tf::StampedTransform transform_baseLink_time;
    try {
      tf_listener_.lookupTransform(base_link_frame_ID_, scan_front_.header.stamp, base_link_frame_ID_,
                                   scan_back_.header.stamp, odom_frame_ID_, transform_baseLink_time); //scan_back_.header.frame_id

      projector.transformLaserScanToPointCloud(base_link_frame_ID_, scan_back_, cloudBackScan_timeOld,
                                               tf_listener_, laser_param_.usableRange, 8);

      pcl_ros::transformPointCloud(base_link_frame_ID_, transform_baseLink_time, cloudBackScan_timeOld,
                                   cloudBackScan);
      ais_pointcloud2_pub_.publish(cloudBackScan);
      //pcl::fromROSMsg(cloudBackScan, PointCloudXYZ);
      pcl_conversions::toPCL(cloudBackScan, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, PointCloudXYZ);

    } catch (tf::TransformException& ex) {
      std::string ns = ros::this_node::getNamespace();
      std::string node_name = ros::this_node::getName();
      ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
      // ROS_ERROR_STREAM(
      //     "Not localized in this step because of tf error (base_link to base_link) and point cloud. Return and continue as normal.");
      return;
    }
    //------------------------------------------------------------------------------
    //  add rear laser data to point cloud
    cloudsize = (PointCloudXYZ.width) * (PointCloudXYZ.height);
    for (int i = 0; i < cloudsize; i++) {
      endPoints.push_back(Vector3f(PointCloudXYZ.points[i].x, PointCloudXYZ.points[i].y, 0.));
    }
  }
  //##############################################################################

  //##############################################################################
  // Getting robot pose from odometry
  // (the interpolated pose when the front reading was taken)
  //##############################################################################
  Transformation3 pose(
      Vector3(transform_odom2baseLink_frontTime.getOrigin().getX(),
              transform_odom2baseLink_frontTime.getOrigin().getY(), 0),
      Quaternion(transform_odom2baseLink_frontTime.getRotation().getX(),
                 transform_odom2baseLink_frontTime.getRotation().getY(),
                 transform_odom2baseLink_frontTime.getRotation().getZ(),
                 transform_odom2baseLink_frontTime.getRotation().getW()));
  //##############################################################################

  //##############################################################################
  // Localization Step
  //##############################################################################
  //cout << "############# LOCALIZATION STEP #############" << endl;
  //cout << "\tOdometry: " << pose.toVector() << endl;
  //cout << "\tEndpoints size: " << endPoints.size() << endl;

  bool updated = false;
  if (endPoints.size() > 0) {
    //cout << "\tUpdating motion: " << endl;
    if (!localizer_.isInitialized()) {
      //cout << "\tLocalizer update State" << endl;
      localizer_.update_state(pose, robotLaser.m_timeStamp);
      localizer_.isInitialized() = true;
    } else {
      localizer_.update_motion(pose);
      updated = localizer_.update_laser(endPoints, pose, robotLaser.m_timeStamp);
      //cout << "\tLocalizer update Laser = " << updated << endl;
    }

  }
  // compute the mean of the particles
  mean = localizer_.getMeanInterp();
  //cout << "\tLocalizer mean: " << mean.toVector() << endl;
  //##############################################################################

  //##############################################################################
  // Create particle cloud msg for the ros (Rviz) and publish
  //##############################################################################
  ParticleVector *particles;
  localizer_.getParticles(particles, false);

  geometry_msgs::PoseArray particleVector;
  geometry_msgs::Pose particlePose;

  static int particleVector_seq = 0;
  particleVector.header.seq = particleVector_seq++;
  particleVector.header.stamp = scan_front_.header.stamp;
  particleVector.header.frame_id = map_frame_ID_;

  for (unsigned int i = 0; i < particles->size(); i++) {
    particlePose.position.x = (*particles)[i].pose._translation.x();
    particlePose.position.y = (*particles)[i].pose._translation.y();
    particlePose.position.z = (*particles)[i].pose._translation.z();
    particlePose.orientation.w = (*particles)[i].pose._rotation.w();
    particlePose.orientation.x = (*particles)[i].pose._rotation.x();
    particlePose.orientation.y = (*particles)[i].pose._rotation.y();
    particlePose.orientation.z = (*particles)[i].pose._rotation.z();
    
    particleVector.poses.push_back(particlePose);
  }

  //ais_particlecloud_pub_.publish(particleVector);  at the end of function
  //##############################################################################

  //##############################################################################
  // Create pose msg of the localized pose and publish (mean of the particles)
  // and make use of scan matcher scan to map
  //##############################################################################
  static int cur_pos_seq = 0;
  geometry_msgs::PoseWithCovarianceStamped cur_pos;
  cur_pos.header.frame_id = map_frame_ID_;
  cur_pos.header.stamp = scan_front_.header.stamp;
  cur_pos.header.seq = cur_pos_seq++;

  localizer_.getCovariance(cur_pos);
  
  if (scanMatcherOn && localizer_.isLocalized()) {
    // scan matcher stuff
    Pose estimatedPose(mean._translation.x(), mean._translation.y(), mean.toVector()[5]);
    Pose correctedPose;
    double smScore = 0.0;
    double** smCovariance = new double*[3];
    for (int i = 0; i < 3; i++)
      smCovariance[i] = new double[3];

    scanMatcher.correlativeScanMatch(endPoints, estimatedPose, correctedPose, smScore);

    double filter = 0.97; // 0.0 ... 1.0

    delta_sm_x = delta_sm_x * filter + ( 1 - filter) * (correctedPose.x - mean._translation.x());
    delta_sm_y = delta_sm_y * filter + ( 1 - filter) * (correctedPose.y - mean._translation.y());
    delta_sm_theta = delta_sm_theta * filter + ( 1 - filter) * (correctedPose.theta - mean.toVector()[5]);

    //std::cerr << "x,y,theta: " << delta_sm_x << ", " << delta_sm_y << ", " << correctedPose.theta << std::endl;

    cur_pos.pose.pose.position.x = delta_sm_x + mean._translation.x();
    cur_pos.pose.pose.position.y = delta_sm_y + mean._translation.y();
    cur_pos.pose.pose.position.z = delta_sm_theta + mean._translation.z();
    cur_pos.pose.pose.orientation = tf::createQuaternionMsgFromYaw(correctedPose.theta);

  } else {
    // use particle filer information
    cur_pos.pose.pose.position.x = mean._translation.x();
    cur_pos.pose.pose.position.y = mean._translation.y();
    cur_pos.pose.pose.position.z = mean._translation.z();
    cur_pos.pose.pose.orientation.w = mean._rotation.w();
    cur_pos.pose.pose.orientation.x = mean._rotation.x();
    cur_pos.pose.pose.orientation.y = mean._rotation.y();
    cur_pos.pose.pose.orientation.z = mean._rotation.z();
  }

  ais_pose_pub_.publish(cur_pos); 
  //##############################################################################


  //##############################################################################
  // Create transform map_frame_ID_ to odom_frame_ID_ and /base_link to /icp_ref
  //##############################################################################
  // transform information map_frame_ID_ to base_link_frame_ID_ into map_frame_ID_ to odom_frame_ID_
  // by using odom_frame_ID_ to base_link_frame_ID_ information
  tf::Pose cur_pos_tf;
  tf::poseMsgToTF(cur_pos.pose.pose, cur_pos_tf);
  tf::Pose diff = cur_pos_tf * transform_odom2baseLink_frontTime.inverse();

	
  tf::Transform transform;
  transform.setOrigin(diff.getOrigin());
  transform.setRotation(diff.getRotation());
  //	br_->sendTransform(tf::StampedTransform(transform, scan_front_.header.stamp, map_frame_ID_, odom_frame_ID_));

  ais_particlecloud_pub_.publish(particleVector); //publish particle cloud
	

  // publish the pose of the odometry o
  tf::poseTFToMsg(diff,cur_pos.pose.pose);

  ais_odom_pose_pub_.publish(cur_pos);
		
}


void Ais_localizer_node::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

  if (!init_map_)
    return;

  if (use_second_laser_ && scan->header.frame_id == back_laser_frame_ID_) {
    scan_back_ = *scan;
    scan_back_new_ = true;
    // cout << "############# Read Rear laser #############" << endl;
  }

	
  if (use_second_laser_ && scan->header.frame_id == front_laser_frame_ID_) {
    scan_front_ = *scan;
    scan_front_new_ = true;
    // cout << "############# Read Front laser #############" << endl;
  }

	
  // if only one laser is in use
  if (!use_second_laser_ && scan->header.frame_id == front_laser_frame_ID_) {
    scan_front_ = *scan;
    scan_front_new_ = true;
  }
	

  if(verbose_){	  
    cerr << "\nUse second Laser: " << use_second_laser_ << endl;
    cerr << "Header id: " << scan->header.frame_id  << endl;
    cerr << "Known frames: "  << "\t" << front_laser_frame_ID_ << ",\t" << back_laser_frame_ID_ << endl;
  }

	
}



std::vector<Vector3f> Ais_localizer_node::endpoints_from_laser(const RobotLaser& laser) {
  std::vector < Vector3f > points;

  double angle = laser.m_parameters.startAngle + laser.m_laserPose.theta;

  for (unsigned int i = 0; i < laser.m_ranges.size(); i++) {
    if (laser.m_ranges[i] < laser.m_parameters.maxRange) {
      double x = laser.m_ranges[i] * cos(angle) + laser.m_laserPose.x;
      double y = laser.m_ranges[i] * sin(angle) + laser.m_laserPose.y;

      points.push_back(Vector3f(x, y, 0.));
    }

    angle += laser.m_parameters.angularResolution;
  }

  return points;
}

std::vector<Vector3f> Ais_localizer_node::local_endpoints_from_laser(const RobotLaser& laser) {
  std::vector < Vector3f > points;

  double angle = laser.m_parameters.startAngle;

  for (unsigned int i = 0; i < laser.m_ranges.size(); i++) {
    if (laser.m_ranges[i] < laser.m_parameters.maxRange) {
      double x = laser.m_ranges[i] * cos(angle);
      double y = laser.m_ranges[i] * sin(angle);

      points.push_back(Vector3f(x, y, 0.));
    }

    angle += laser.m_parameters.angularResolution;
  }

  return points;
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "squirrel_localizer");
  Ais_localizer_node ais_localizer_node;

  ros::Rate rate(10);
  while(ros::ok()){
    ros::spinOnce();
    ais_localizer_node.localize();
    rate.sleep();
  }
}

// 
// LocalizerApp.cpp ends here
