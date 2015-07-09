#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <algorithm>

class kinect_test
{
 public:
  kinect_test( void )
  {
    sub_ = nh_.subscribe("/rgbdscan", 1, &kinect_test::getMaxZ, this);
  };
  
  virtual ~kinect_test( void ) {};

 private:
  void getMaxZ( const sensor_msgs::PointCloud2::ConstPtr& pcl_msg )
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_k, cloud_b;
    pcl::fromROSMsg(*pcl_msg, cloud_k);
    
    try {
      tfl_.waitForTransform("/base_link", "/kinect_depth_optical_frame", ros::Time::now(), ros::Duration(0.25));
      pcl_ros::transformPointCloud("/base_link", cloud_k, cloud_b, tfl_);
    } catch( tf::LookupException& ex ) {
      ROS_ERROR("Lookup Error: %s", ex.what());
      return;
    } catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s", ex.what());
      return;
    } catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s", ex.what());
      return;
    }

    double z_max = -1000;
    for (unsigned int i=0; i<cloud_b.size(); ++i) {
      z_max = cloud_b[i].z > z_max ? cloud_b[i].z : z_max;
    }
    std::cout << "max z: " << z_max << std::endl;
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  tf::TransformListener tfl_;
};

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "test_kinect");

  kinect_test t;
  
  ros::Rate lr(10);
  while ( ros::ok() ) {     
    ros::spinOnce();
    lr.sleep();
  }
  
  return 0;
}
