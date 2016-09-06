/*
 * 6D localization for humanoid robots
 *
 * Copyright 2009-2012 Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/humanoid_localization
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SQUIRREL_3D_LOCALIZER_SQUIRRELLOCALIZER_H_
#define SQUIRREL_3D_LOCALIZER_SQUIRRELLOCALIZER_H_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <squirrel_3d_localizer_msgs/SetMap.h>
#include <squirrel_3d_localizer_msgs/ToggleSensors.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>

#include <squirrel_3d_localizer/EndpointModel.h>
#include <squirrel_3d_localizer/MotionModel.h>
#include <squirrel_3d_localizer/ObservationModel.h>
#include <squirrel_3d_localizer/RaycastingModel.h>
#include <squirrel_3d_localizer/SquirrelLocalizerDefs.h>

#include <octomap/octomap.h>

#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>

#include <ctime>

namespace squirrel_3d_localizer {

static inline void getRP(
    const geometry_msgs::Quaternion& msg_q, double& roll, double& pitch) {
  tf::Quaternion bt_q;
  tf::quaternionMsgToTF(msg_q, bt_q);
  double useless_yaw;
  tf::Matrix3x3(bt_q).getRPY(roll, pitch, useless_yaw);

  if (std::abs(useless_yaw) > 0.00001)
    ROS_WARN_STREAM(
        ros::this_node::getName()
        << ": Non-zero yaw in IMU quaterion is ignored");
}

class SquirrelLocalizer {
 public:
  // some typedefs
  // typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

 public:
  SquirrelLocalizer(unsigned randomSeed);
  virtual ~SquirrelLocalizer();

  /// callback for laser scanner data
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  /// callback for depth camera data
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  /// callback for synchronized laser scanner and depth camera data
  void synchronizedCallback(
      const sensor_msgs::LaserScan::ConstPtr& scanMsg,
      const sensor_msgs::PointCloud2::ConstPtr& cloudMsg);
  /// reinitialization from pose
  void initPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  /// global reinitialization
  bool globalLocalizationCallback(
      std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  /// pause localization by message
  void pauseLocalizationCallback(const std_msgs::BoolConstPtr& msg);
  /// pause localization by service call
  bool pauseLocalizationSrvCallback(
      std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  /// unpause localization by service call
  bool resumeLocalizationSrvCallback(
      std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  /// toggle sensors on/off by service call
  bool toggleSensorsSrvCallback(
      squirrel_3d_localizer_msgs::ToggleSensors::Request& req,
      squirrel_3d_localizer_msgs::ToggleSensors::Response& res);
  /// reset map via service call
  bool resetMapSrvCallback(
      squirrel_3d_localizer_msgs::SetMap::Request& req,
      squirrel_3d_localizer_msgs::SetMap::Response& res);

  /**
   * Importance sampling from m_particles according to weights,
   * resets weight to 1/numParticles. Uses low variance sampling
   *
   * @param numParticles how many particles to sample, 0 (default): keep size of
   * particle distribution
   */
  void resample(unsigned numParticles = 0);
  /// Returns index of particle with highest weight (log or normal scale)
  unsigned getBestParticleIdx() const;
  /// Returns the 6D pose of a particle
  tf::Pose getParticlePose(unsigned particleIdx) const;
  /// Returns the 6D pose of the best particle (highest weight)
  tf::Pose getBestParticlePose() const;
  /// Returns the 6D pose of the weighted mean particle
  tf::Pose getMeanParticlePose() const;

  /// function call for global initialization (called by
  /// globalLocalizationCallback)
  void initGlobal();

  // needed for pointcloud callback (from OctomapServer)
  static void filterGroundPlane(
      const PointCloud& pc, PointCloud& ground, PointCloud& nonground,
      double groundFilterDistance, double groundFilterAngle,
      double groundFilterPlaneDistance);

 protected:
  /**
   * General reset of the filter:
   * sets pose around initial pose (or truepose, if requested) and resets
   * the internal state. Calls initPoseCallback().
   */
  void reset();

  // converts particles into PoseArray and publishes them for visualization
  void publishPoseEstimate(const ros::Time& time, bool publish_eval);

  /**
   * Normalizes the weights and transforms from log to normal scale
   * m_minWeight gives the lower bound for weight (normal scale).
   * No adjustment will be done for minWeight = 0 (default)
   */
  void normalizeWeights();

  /// cumulative weight of all particles (=1 when normalized)
  double getCumParticleWeight() const;

  /**
   * nEff - returns the number of effective particles = 1/sum(w_i^2)
   *
   * Needed for selective resampling (Doucet 98, Arulampalam 01), when nEff <
   *n/2
   **/
  double nEff() const;

  /**
   * Converts particles into log scale
   */
  void toLogForm();

  /**
   * Returns the IMU message with stamp closest to a given stamp.
   * @param[in] stamp Timestamp to search.
   * @param[out] imuStamp Stamp of closest IMU message (or interpolation of two
   * IMU messages).
   * @param[out] angleX Interpolated roll angle.
   * @param[out] angleY Interpolated pitch angle.
   * @return Success.
   */
  bool getImuMsg(
      const ros::Time& stamp, ros::Time& imuStamp, double& angleX,
      double& angleY) const;

  /**
   * Prepares a LaserScan msg to be integrated into the observations model.
   * Filters
   * near range measurements out and creates a sparse point cloud (out of
   * m_numSensorBeams points)
   *
   */
  void prepareLaserPointCloud(
      const sensor_msgs::LaserScan::ConstPtr& laser, PointCloud& pc,
      std::vector<float>& ranges) const;

  void prepareLaserPointCloud(
      const sensor_msgs::LaserScan::ConstPtr& laser,
      const std::string& targetFrame, PointCloud& pc,
      std::vector<float>& ranges);

  /**
   * Prepares a PointCloud msg to be integrated into the observations model.
   * Filters
   * near range, floor and subsamples a sparse point cloud (out of
   * m_numSensorBeams points)
   *
   */
  void prepareGeneralPointCloud(
      const sensor_msgs::PointCloud2::ConstPtr& msg, PointCloud& pc,
      std::vector<float>& ranges) const;

  int filterUniform(
      const PointCloud& cloud_in, PointCloud& cloud_out, int numSamples) const;

  void voxelGridSampling(
      const PointCloud& pc, pcl::PointCloud<int>& sampledIndices,
      double searchRadius) const;

  bool isAboveMotionThreshold(const tf::Pose& odomTransform);

  bool localizeWithMeasurement(
      const PointCloud& pc_filtered, const std::vector<float>& ranges,
      double max_range);

  void constrainMotion(const tf::Pose& odomPose);

  void timerCallback(const ros::TimerEvent& e);

  unsigned computeBeamStep(unsigned numBeams) const;

  /**
   * Initializes z, roll and pitch values either from parameters
   * or fromo real (odom) values
   */
  void initZRP(double& z, double& roll, double& pitch);

  bool lookupPoseHeight(const ros::Time& t, double& poseHeight) const;

  EngineT m_rngEngine;
  /// standard normal distribution
  NormalGeneratorT m_rngNormal;
  /// uniform distribution [0:1]
  UniformGeneratorT m_rngUniform;
  boost::shared_ptr<MotionModel> m_motionModel;
  boost::shared_ptr<ObservationModel> m_observationModel;
  boost::shared_ptr<MapModel> m_mapModel;

  std::string m_nodeName;
  ros::NodeHandle m_nh, m_privateNh;
  ros::Subscriber m_pauseIntegrationSub;

  message_filters::Subscriber<sensor_msgs::LaserScan>* m_laserSub;
  tf::MessageFilter<sensor_msgs::LaserScan>* m_laserFilter;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* m_pointCloudFilter;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>*
      m_initPoseSub;
  tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseFilter;

  // synchronization of messages
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::LaserScan, sensor_msgs::PointCloud2>
      ApprxTimePolicy;
  message_filters::Synchronizer<ApprxTimePolicy>* synchronizer_;

  ros::Publisher m_posePub, m_poseEvalPub, m_poseOdomPub, m_poseTruePub,
      m_poseArrayPub, m_bestPosePub, m_nEffPub, m_filteredPointCloudPub;
  ros::Subscriber m_imuSub;
  ros::ServiceServer m_globalLocSrv, m_pauseLocSrv, m_resumeLocSrv,
      m_toggleSensorsSrv;
  tf::TransformListener m_tfListener;
  tf::TransformBroadcaster m_tfBroadcaster;
  ros::Timer m_timer;

  bool m_verbose;
  bool m_printPointCloudSubscription;
  bool m_printLaserSubscription;
  bool m_printSynchronizedSubscription;

  std::string m_odomFrameId;
  std::string m_targetFrameId;
  std::string m_baseFrameId;
  std::string m_baseFootprintId;
  std::string m_globalFrameId;

  bool m_useDepthCamera, m_useLaserScanner;

  bool m_useRaycasting;
  bool m_initFromTruepose;
  bool m_useLastPose;
  int m_numParticles;
  double m_sensorSampleDist;

  double m_nEffFactor;
  double m_minParticleWeight;
  Vector6d m_initPose;      // fixed init. pose (from params)
  Vector6d m_initNoiseStd;  // Std.dev for init. pose
  bool
      m_initPoseRealZRP;  // override z, roll, pitch with real values from robot

  double m_filterMaxRange;
  double m_filterMinRange;

  Particles m_particles;
  int m_bestParticleIdx;
  tf::Pose m_odomPose;  // incrementally added odometry pose (=dead reckoning)
  tf::Pose m_bestParticlePose;
  geometry_msgs::PoseArray m_poseArray;  // particles as PoseArray
  boost::circular_buffer<sensor_msgs::Imu> m_lastIMUMsgBuffer;

  bool m_bestParticleAsMean;
  bool m_receivedSensorData;
  bool m_initialized;
  bool m_initGlobal;
  bool m_paused;
  bool m_syncedTruepose;

  double m_observationThresholdTrans;
  double m_observationThresholdRot;
  double m_observationThresholdHeadYawRot;
  double m_observationThresholdHeadPitchRot;
  double m_temporalSamplingRange;
  double m_transformTolerance;
  ros::Time m_lastLaserTime;
  ros::Time m_lastPointCloudTime;

  // PointCloud parameters

  bool m_groundFilterPointCloud;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;
  double m_sensorSampleDistGroundFactor;

  /// sensor data last integrated at this odom pose, to check if moved enough
  /// since then
  tf::Pose m_lastLocalizedPose;
  tf::StampedTransform m_latest_transform;

  /// absolute, summed yaw angle since last measurement integraton
  double m_headYawRotationLastScan;
  /// absolute, summed pitch angle since last measurement integraton
  double m_headPitchRotationLastScan;

  bool m_useIMU;  ///< True = use IMU for initialization and observation models,
                  /// false = use orientation from odometry
  bool m_constrainMotionZ;   /// < True = do not estimate height, directly use
                             /// odometry pose
  bool m_constrainMotionRP;  /// < True = do not estimate roll and pitch,
                             /// directly use odometry pose

  // timer stuff
  bool m_useTimer;
  double m_timerPeriod;
};

}  // namespace squirrel_3d_localizer

#endif /* SQUIRREL_3D_LOCALIZER_SQUIRRELLOCALIZER_H_ */
