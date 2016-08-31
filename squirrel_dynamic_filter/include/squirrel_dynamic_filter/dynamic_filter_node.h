#ifndef DYNAMICFILTERNODE_H
#define DYNAMICFILTERNODE_H
#include <ros/ros.h>
#include "structure.h"
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "EuclideanClusteringMesh.h"
#include "FeatureSHOT.h"
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/impl/correspondence_estimation.hpp>
#include "squirrel_dynamic_filter_msgs/CloudMsg.h"
using namespace Eigen;

typedef Matrix<uint8_t, Dynamic, Dynamic> MatrixXint;
class DynamicFilter
{
private:
 message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
 ros::Subscriber cloud_sub_1;
 ros::Publisher pub_tracked_gt;
 tf::TransformListener tf_;
 tf::MessageFilter <sensor_msgs::PointCloud2> * tf_filter_;
 ros::NodeHandle n_;
 Frame frame_1;
 Frame frame_2;
 Isometry3D odometry_diff;
 std::vector<correspondences>c_vec;
 std::vector<correspondences>c_vec_actual;
 bool is_first_frame;
 int frame_counter;
 PointCloud::Ptr cloud_input;
 PointCloud::Ptr cloud_input_objects;
 pcl::VoxelGrid<Point> sor;
 std::stringstream ss;
 std::vector< std::vector<int> > neighbours;
 void preprocessing(Frame &frame);
 void EstimateFeature(Frame &frame);
 void EstimateCorrespondence(const float sampling_radius,const float max_motion, std::vector<int>&index_query, std::vector<int> &index_match);
 void EstimateMotion(const std::vector <int> &index_query,const std::vector <int> &index_match);
 void EstimateMotion2(const std::vector <int> &index_query,const std::vector <int> &index_match);
 void EstimateCorrespondencePoint(const float radius,const float sampling_radius,int number_correspondences,std::vector<int>&index_query, std::vector<int> &index_match);
 void filter_correspondences(const float neighbour_radius,const float covariance_value,const float score_threshold);
 void optimize_keypoint(const PointCloud::Ptr& kp,const std::vector<int>&index_match,const std::vector<int>&index_query,std::vector<Isometry3D> &motion_kp);

 void sample(const Frame frame,const float radius, const float threshold,std::vector<int> &sampled_finite);
 Eigen::Matrix4f trans;

 float down_sampling_radius;
 float feature_cluster_max_length;
 int feature_cluster_max_points;
 float normal_radius;
 float feature_radius;
 float sampling_radius;
 float feature_score_threshold;
 float filter_radius;
 float filter_variance;
 float keypoint_radius;
 float max_motion;
 double feature_time;
 double correspondence_time;
 double motion_time;
 double total_time;
 measure_time start;
 measure_time end;
 TimeDiff time_diff;
 measure_time start_total;
 measure_time end_total;
 ofstream time_write;





public:
 DynamicFilter();
 string output_folder;
 void cloudMsgCallback(const squirrel_dynamic_filter_msgs::CloudMsg& sensor_msg);

 ofstream myfile_odom;

};
#endif
