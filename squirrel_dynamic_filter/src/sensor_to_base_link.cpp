// Copyright (c) 2016-2017, Ayush Dewan and Wolfram Burgard
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
