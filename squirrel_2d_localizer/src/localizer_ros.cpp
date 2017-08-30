// The MIT License (MIT)
//
// Copyright (c) 2016-2017 Federico Boniardi and Wolfram Burgard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "squirrel_2d_localizer/localizer_ros.h"

#include <ros/package.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>

#include <angles/angles.h>

#include <fstream>
#include <random>
#include <stdexcept>
#include <string>

namespace squirrel_2d_localizer {

LocalizerROS::LocalizerROS()
    : node_name_(ros::this_node::getName()),
      update_laser_params_(true),
      initial_localization_counter_(0) {
  ros::NodeHandle nh("~"), gnh;
  // frames.
  nh.param<std::string>("map_frame", map_frame_id_, "map");
  nh.param<std::string>("odom_frame", odom_frame_id_, "odom");
  nh.param<std::string>("robot_frame", robot_frame_id_, "base_link");
  // Publish extra frames.
  nh.param<bool>("publish_extra_tf", publish_extra_tf_, false);
  nh.param<std::string>("extra_parent_frame_id", extra_parent_frame_id_, "map");
  nh.param<std::string>("extra_child_frame_id", extra_child_frame_id_, "orig");
  // initialization
  nh.param<bool>("use_last_pose", use_last_pose_, false);
  nh.param<double>("init_pose_x", init_x_, 0.);
  nh.param<double>("init_pose_y", init_y_, 0.);
  nh.param<double>("init_pose_a", init_a_, 0.);
  nh.param<bool>("use_twist_correction", use_twist_correction_, false);
  if (use_twist_correction_)
    twist_correction_.reset(new TwistCorrectionROS);
  // localizer parameters.
  ros::NodeHandle loc_nh("~/localizer");
  Localizer::Params loc_param;
  loc_nh.param<int>("num_particles", loc_param.num_particles, 1000);
  loc_nh.param<double>("min_lin_update", loc_param.min_lin_update, 0.5);
  loc_nh.param<double>("min_ang_update", loc_param.min_ang_update, 0.5);
  loc_nh.param<double>("init_stddev_x", loc_param.init_stddev_x, 0.5);
  loc_nh.param<double>("init_stddev_y", loc_param.init_stddev_y, 0.5);
  loc_nh.param<double>("init_stddev_a", loc_param.init_stddev_a, 0.5);
  localizer_.reset(new Localizer(loc_param));
  // motion model parameters.
  ros::NodeHandle mm_nh("~/motion_model");
  MotionModel::Params motion_param;
  mm_nh.param<double>("noise_xx", motion_param.noise_xx, 1.);
  mm_nh.param<double>("noise_xy", motion_param.noise_xy, 0.);
  mm_nh.param<double>("noise_xa", motion_param.noise_xa, 0.);
  mm_nh.param<double>("noise_yy", motion_param.noise_yy, 1.);
  mm_nh.param<double>("noise_ya", motion_param.noise_ya, 0.);
  mm_nh.param<double>("noise_aa", motion_param.noise_aa, 1.);
  mm_nh.param<double>("noise_magnitude", motion_param.noise_magnitude, 0.5);
  std::unique_ptr<MotionModel> motion_model(new MotionModel(motion_param));
  // likelihood fields paramters.
  ros::NodeHandle lf_nh("~/latent_model_likelihood_field");
  LatentModelLikelihoodField::Params lmlf_param;
  lf_nh.param<double>("uniform_hit", lmlf_param.uniform_hit, 0.1);
  lf_nh.param<double>("observation_sigma", lmlf_param.observation_sigma, 1.);
  std::unique_ptr<LatentModelLikelihoodField> likelihood_field(
      new LatentModelLikelihoodField(lmlf_param));
  // laser model params.
  ros::NodeHandle lm_nh("~/laser_model");
  LaserModel::Params lm_param;
  lm_nh.param<double>(
      "beam_min_distance", lm_param.endpoints_min_distance, 0.1);
  std::unique_ptr<LaserModel> laser_model(new LaserModel(lm_param));
  // init grid map
  GridMap::Params map_params;
  nav_msgs::GetMap get_map;
  if (ros::service::waitForService("/static_map", ros::Duration(60.))) {
    if (ros::service::call("/static_map", get_map)) {
      map_params.resolution = get_map.response.map.info.resolution;
      map_params.height     = get_map.response.map.info.height;
      map_params.width      = get_map.response.map.info.width;
      map_params.origin     = ros_conversions::fromROSMsgTo<Pose2d>(
          get_map.response.map.info.origin);
      ROS_INFO(
          "%s: Received map %ld x %ld (%f m/px)", node_name_.c_str(),
          map_params.height, map_params.width, map_params.resolution);
    } else {
      ROS_ERROR_STREAM(
          node_name_ << ": Unable to fetch the map. Shutting down.");
      ros::shutdown();
    }
  } else {
    ROS_ERROR_STREAM(
        node_name_ << ": Service [/static_map] not advertised. Shutting down.");
    ros::shutdown();
  }
  // Wait for annoying ROS crap to start up. Apparently no better solution.
  ros::Duration(1.0).sleep();
  // Initialize grid map and likelihood fields.
  std::unique_ptr<GridMap> grid_map(new GridMap(map_params));
  grid_map->initialize(get_map.response.map.data);
  likelihood_field->initialize(*grid_map);
  ROS_INFO_STREAM(node_name_ << ": Initialized LikelihoodField.");
  // initialize objects;
  localizer_->initialize(grid_map, likelihood_field, laser_model, motion_model);
  while (!lookupOdometry(ros::Time(0), ros::Duration(1.0), &tf_o2r_))
    ROS_WARN_STREAM(node_name_ << ": Trying to initialize odometry.");
  ROS_INFO_STREAM(node_name_ << ": Odometry initialized.");
  if (use_last_pose_) {
    ros::NodeHandle lp_nh("~/last_pose");
    lp_nh.param<double>("x", init_x_, 0.);
    lp_nh.param<double>("y", init_y_, 0.);
    lp_nh.param<double>("a", init_a_, 0.);
  }
  localizer_->resetPose(Pose2d(init_x_, init_y_, init_a_));
  // Advertise publishers, subscribers and services.
  scan_sub_     = gnh.subscribe("/scan", 1, &LocalizerROS::laserCallback, this);
  initpose_sub_ = gnh.subscribe(
      "/initialpose", 1, &LocalizerROS::initialPoseCallback, this);
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
  particles_pub_ = nh.advertise<geometry_msgs::PoseArray>("particles", 1);
  gloc_srv_      = nh.advertiseService(
      "globalLocalization", &LocalizerROS::globalLocalizationCallback, this);
  // Broadcast initial state.
  const ros::Time now = ros::Time::now();
  publishTransform(now);
  publishParticles(now);
  publishPoseWithCovariance(now);
}

LocalizerROS::~LocalizerROS() {
  const std::string last_pose_filename =
      ros::package::getPath("squirrel_2d_localizer") +
      std::string("/config/last_pose.yaml");
  std::ofstream fout(last_pose_filename.c_str());
  if (fout.is_open() && fout.good()) {
    const Pose2d& pose = localizer_->pose();
    fout << "last_pose: " << std::endl
         << "  x: " << pose[0] << std::endl
         << "  y: " << pose[1] << std::endl
         << "  a: " << angles::normalize_angle(pose[2]);
    fout.close();
  } else {
    ROS_ERROR_STREAM(node_name_ << ": Unable to dump robot last pose.");
  }
}

void LocalizerROS::spin(double hz) {
  for (ros::Rate lr(hz); ros::ok(); lr.sleep()) {
    try {
      ros::spinOnce();
      publishTransform(ros::Time::now());
    } catch (const std::runtime_error& err) {
      ROS_ERROR_STREAM(node_name_ << ": " << err.what());
    }
  }
}

void LocalizerROS::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO_STREAM_ONCE(node_name_ << ": Subscribing to laser scan.");
  ros::Time scan_time = msg->header.stamp;
  // update laser paramters from actual scan msg,
  if (update_laser_params_) {
    tf::StampedTransform tf_r2l;
    try {
      tfl_.lookupTransform(
          robot_frame_id_, msg->header.frame_id, ros::Time(0), tf_r2l);
    } catch (tf::TransformException& ex) {
      ROS_WARN_STREAM(
          node_name_ << ": Unable to retrieve laser pose. Trying again.");
      return;
    }
    // update parameters in the laser model.
    LaserModel::Params& laser_params = localizer_->laserModel()->params();
    laser_params.range_min           = msg->range_min;
    laser_params.range_max           = msg->range_max;
    laser_params.angle_min           = msg->angle_min;
    laser_params.angle_max           = msg->angle_max;
    laser_params.tf_r2l  = ros_conversions::fromTFMsgTo<Pose2d>(tf_r2l);
    update_laser_params_ = false;
    return;
  }
  // update filter.
  std::unique_lock<std::mutex> lock(update_mtx_);
  tf::StampedTransform tf_o2r_new;
  if (!lookupOdometry(scan_time, ros::Duration(0.05), &tf_o2r_new))
    return;
  const Pose2d& motion =
      ros_conversions::fromTFMsgTo<Pose2d>(tf_o2r_.inverse() * tf_o2r_new);
  const Pose2d& correction = use_twist_correction_
                                 ? twist_correction_->correction(scan_time)
                                 : Pose2d(0., 0., 0.);
  const bool force_update = initial_localization_counter_++ < 5;
  if (localizer_->updateFilter(motion, msg->ranges, correction, force_update)) {
    tf_o2r_ = tf_o2r_new;
    publishParticles(scan_time);
    publishPoseWithCovariance(scan_time);
  }
}

void LocalizerROS::initialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  ros::Time now;
  while (!lookupOdometry(now = ros::Time::now(), ros::Duration(0.1), &tf_o2r_))
    ROS_WARN_STREAM(node_name_ << ": Trying to reinitialize odometry.");
  localizer_->resetPose(ros_conversions::fromROSMsgTo<Pose2d>(msg->pose.pose));
  publishParticles(now);
  publishPoseWithCovariance(now);
  initial_localization_counter_ = 0;
}

bool LocalizerROS::globalLocalizationCallback(
    squirrel_2d_localizer_msgs::GlobalLocalization::Request& req,
    squirrel_2d_localizer_msgs::GlobalLocalization::Response& res) {
  // Get the gridmap size.
  const auto gridmap = localizer_->gridMap();
  double min_x, max_x, min_y, max_y;
  gridmap->boundingBox(&min_x, &max_x, &min_y, &max_y);
  // Create the sampler.
  std::mt19937 rnd(std::rand());
  std::uniform_real_distribution<double> rand_x(min_x, max_x),
      rand_y(min_x, max_y), rand_a(-M_PI, M_PI);
  // Sample new particles.
  const size_t nparticles = localizer_->params().num_particles;
  std::vector<Particle> new_particles;
  new_particles.reserve(nparticles);
  const ros::Time start = ros::Time::now();
  for (int i, j; new_particles.size() < nparticles;) {
    if (ros::Time::now() - start > req.timeout) {
      res.sampling_time         = req.timeout;
      res.num_sampled_particles = new_particles.size();
      // Resampling timed out.
      return false;
    }
    const Pose2d particle_pose(rand_x(rnd), rand_y(rnd), rand_a(rnd));
    const double particle_weight = 1. / nparticles;
    gridmap->pointToIndices(particle_pose.translation(), &i, &j);
    if (gridmap->inside(i, j) && !gridmap->unknown(i, j) &&
        gridmap->at(i, j) <= 0.25)
      new_particles.emplace_back(particle_pose, particle_weight);
  }
  const ros::Time& now      = ros::Time::now();
  res.sampling_time         = now - start;
  res.num_sampled_particles = nparticles;
  // Reinitialize the localizer.
  localizer_->resetParticles(new_particles);
  publishParticles(now);
  // Resampling was successful.
  return true;
}

bool LocalizerROS::lookupOdometry(
    const ros::Time& stamp, const ros::Duration& timeout,
    tf::StampedTransform* tf_o2r) {
  try {
    tfl_.waitForTransform(odom_frame_id_, robot_frame_id_, stamp, timeout);
    tfl_.lookupTransform(odom_frame_id_, robot_frame_id_, stamp, *tf_o2r);
  } catch (const tf::TransformException& ex) {
    ROS_ERROR_STREAM(node_name_ << ": " << ex.what());
    return false;
  }
  return true;
}

void LocalizerROS::publishTransform(const ros::Time& stamp) {
  std::unique_lock<std::mutex> lock(update_mtx_);
  tf::Transform tf_m2o,
      tf_m2r = ros_conversions::toTFMsgFrom<Pose2d>(localizer_->pose());
  tf_m2o     = tf_m2r * tf_o2r_.inverse();
  tfb_.sendTransform(
      tf::StampedTransform(tf_m2o, stamp, map_frame_id_, odom_frame_id_));
  if (publish_extra_tf_)
    extra_tfb_.sendTransform(tf::StampedTransform(
        tf_m2o, stamp, extra_parent_frame_id_, extra_child_frame_id_));
}

void LocalizerROS::publishParticles(const ros::Time& stamp) {
  const std::vector<Particle>& particles = localizer_->particles();
  geometry_msgs::PoseArray msg;
  msg.header.frame_id = map_frame_id_;
  msg.header.stamp    = stamp;
  msg.poses.reserve(particles.size());
  for (size_t i = 0; i < particles.size(); ++i)
    msg.poses.emplace_back(
        ros_conversions::toROSMsgFrom<Pose2d>(particles[i].pose));
  particles_pub_.publish(msg);
}

void LocalizerROS::publishPoseWithCovariance(const ros::Time& stamp) {
  const Pose2d& pose         = localizer_->pose();
  const Eigen::Matrix3d& cov = localizer_->covariance();
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = map_frame_id_;
  msg.header.stamp    = stamp;
  msg.pose.pose       = ros_conversions::toROSMsgFrom<Pose2d>(pose);
  for (size_t i = 0, ei; i < 3; ++i) {
    ei = i < 3 ? i : 5;
    for (size_t j = 0, ej; j < 3; ++j) {
      ej                               = j < 3 ? j : 5;
      msg.pose.covariance[6 * ei + ej] = cov(i, j);
    }
  }
  pose_pub_.publish(msg);
}

}  // namespace squirrel_2d_localizer
