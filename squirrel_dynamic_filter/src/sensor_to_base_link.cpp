#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <fstream>

/////File used for listening to transform between sensor optical frame and
//base_link. Required for removing the ground plane


using namespace std;
int main(int argc, char** argv)
{
 ros::init(argc, argv, "my_tf_listener");
 ros::NodeHandle node;
 tf::TransformListener listener;
 ros::Rate rate(10.0);
 string input_folder;
 
 bool transform_found;
 node.getParam("/InputFolder",input_folder);
 node.getParam("/TransformFound",transform_found);

 std::stringstream ss;

 ss << input_folder << "/params/sensor_to_base_link.csv";

 ofstream myfile(ss.str().c_str());
 ROS_INFO("%s","Listening to base_link and kinect_rgb_optical_frame transforms");
 while (node.ok() && !transform_found)
 {
  tf::StampedTransform transform;
  try{
      listener.lookupTransform("/base_link", "/kinect_rgb_optical_frame",  
                               ros::Time(0), transform);
      
      tf::Vector3 pos = transform.getOrigin();
      tf::Quaternion rot = transform.getRotation();
      transform_found = true;

      myfile << pos[0] << "," << pos[1] << "," << pos[2] << "," << rot[0] << "," << rot[1] << "," << rot[2] << "," << rot[3];

      myfile.close();

      node.setParam("/TransformFound",transform_found);
    }
  catch (tf::TransformException ex){
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
  }
  rate.sleep();
 }


 return 0;
};
