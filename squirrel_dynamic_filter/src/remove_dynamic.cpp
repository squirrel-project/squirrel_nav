#include "datatypes_squirrel.h"
#include <pcl/visualization/pcl_visualizer.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>

bool close_viewer;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "n")
  {
    close_viewer = true;


  }
}


using namespace std;

int main(int argc, char** argv)
{

 ros::init(argc, argv, "squirrel_dynamic_filter_viz");
 ros::NodeHandle nh;
 int start_frame = atoi(argv[1]);
 int end_frame = atoi(argv[2]);

 string folder = argv[3];

 pcl::PCDReader reader;

 IntensityCloud::Ptr cloud(new IntensityCloud);
 RGBCloud ground;
 RGBCloud::Ptr non_movable_cloud(new RGBCloud);
 std::stringstream ss;

 ifstream myfile;
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
 std::vector < std::vector<float> > motion_trans;

 ss.str("");

 ss << folder << "/odometry.csv";

 string line;
 myfile.open(ss.str());
 if (myfile.is_open())
 {
  while ( getline (myfile,line) )
  {
    std::vector<float>trans(7,0.0);
    for(int i = 0 ; i<7;i++)
    {

      std::size_t found = line.find(",");
      if(i==0)
        trans[0] = atof(line.substr(0,found).c_str());
      if(i==1)
        trans[1] = atof(line.substr(0,found).c_str());
      if(i==2)
        trans[2] = atof(line.substr(0,found).c_str());
      if(i==3)
        trans[3] = atof(line.substr(0,found).c_str());
      if(i==4)
        trans[4] = atof(line.substr(0,found).c_str());
      if(i==5)
        trans[5] = atof(line.substr(0,found).c_str());
      if(i==6)
        trans[6] = atof(line.substr(0,found).c_str());
      string line_new=line.substr(found+1);
      line=line_new;

    }

    motion_trans.push_back(trans);

  }
 }

 myfile.close();

 ros::Publisher pub_non_movable = nh.advertise<sensor_msgs::PointCloud2>("/non_movable",10);//Publising the filtered pointcloud
 ros::Publisher pub_dynamic = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_cloud",10);//Publising the filtered pointcloud
 ros::Publisher pub_movable = nh.advertise<sensor_msgs::PointCloud2>("/movable",10);//Publising the filtered pointcloud
 tf::TransformBroadcaster br;
 tf::Transform transform_map_base_link;

 PointCloud movable_cloud;

 for(size_t i = start_frame; i < end_frame; ++i)
 {
  cloud->points.clear();
  non_movable_cloud->points.clear();
  movable_cloud.points.clear();

  cout << i << endl;
  ss.str("");
  ss << folder << "cloud_save_robust_a_" << i << ".pcd";

  reader.read(ss.str(),*cloud);
  ss.str("");
  ss << folder << "ground_" << i << ".pcd";

  reader.read(ss.str(),ground);
  ss.str("");
  ss << folder << "static_cloud_" << i << ".pcd";

  reader.read(ss.str(),*non_movable_cloud);

  ss.str("");
  ss << folder << "intensity_z_" << i << ".csv";

  myfile.open(ss.str().c_str());
  int counter = 0;
  if (myfile.is_open())
  {
   while ( getline (myfile,line) )
   {
    if(atof(line.c_str()) < 0.02)
    {
      pcl::PointXYZ pt;
      pt.x = cloud->points[counter].x;
      pt.y = cloud->points[counter].y;
      pt.z = cloud->points[counter].z;

      movable_cloud.points.push_back(pt);

    }



     cloud->points[counter].intensity = atof(line.c_str());
  //  else
  //   cloud->points[counter].intensity = 0.1;

    counter +=1;
   }
  }

  myfile.close();

  ss.str("");
  ss << folder << "map_base_link_" << i << ".csv";


  myfile.open(ss.str().c_str());

  std::vector<float>map_base_link;
  if (myfile.is_open())
  {
   while ( getline (myfile,line) )
   {
     map_base_link.push_back(atof(line.c_str()));


   }
  }

  cout << map_base_link.size() << endl;


  for(auto &point:non_movable_cloud->points)
  {
  /* point.r = 153;*/
   //point.g = 50;
   //point.b = 204;

   point.r = 255;
   point.g = 255;
   point.b = 255;

  }


  for(auto &point:ground.points)
  {
   point.r = 255.0;
   point.g = 255.0;
   point.b = 255.0;
   non_movable_cloud->points.push_back(point);
  }

  myfile.close();
  pcl::PointXYZI min,max;
  min.x = max.x = 0.0;
  min.y = max.y = 0.0;
  min.z = max.z = 0.0;
  min.intensity = 0;
  max.intensity = 1.0;
//  Eigen::Vector3f trans(motion_trans[i][0],motion_trans[i][1],motion_trans[i][2]);

//  cout << trans.lpNorm<2>() << "," << 2 * acos(motion_trans[i][6]) << endl;
  cloud->points.push_back(min);
  cloud->points.push_back(max);

  non_movable_cloud->width = non_movable_cloud->points.size();
  non_movable_cloud->height = 1;

  movable_cloud.width = movable_cloud.points.size();
  movable_cloud.height = 1;


  cloud->width = cloud->points.size();
  cloud->height = 1;

  transform_map_base_link.setOrigin(tf::Vector3(map_base_link[0],map_base_link[1],map_base_link[2]));
  tf::Quaternion q(map_base_link[3],map_base_link[4],map_base_link[5],map_base_link[6]);
  transform_map_base_link.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform_map_base_link, ros::Time::now(), "map", "base_link"));//publish the tf corresponding to points


  sensor_msgs::PointCloud2 non_movable_cloud_msg;
  pcl::toROSMsg(*non_movable_cloud,non_movable_cloud_msg);
  non_movable_cloud_msg.header.frame_id = "/base_link";
  pub_non_movable.publish(non_movable_cloud_msg);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud,cloud_msg);
  cloud_msg.header.frame_id = "/base_link";
  pub_dynamic.publish(cloud_msg);

  sensor_msgs::PointCloud2 movable_cloud_msg;
  pcl::toROSMsg(movable_cloud,movable_cloud_msg);
  movable_cloud_msg.header.frame_id = "/base_link";
  pub_movable.publish(movable_cloud_msg);


  ros::Duration(0.4).sleep();



  /*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_non(non_movable_cloud);*/
  //viewer->addPointCloud <pcl::PointXYZRGB> (non_movable_cloud,rgb_non,"cloudrgb");


  //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgb(cloud,"intensity");
  //viewer->addPointCloud<pcl::PointXYZI> (cloud,rgb,"cloud");
  //std::clock_t start;
  //double duration;
  //start = std::clock();
  //close_viewer = false;
  //viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloudrgb");


  //pcl::visualization::Camera cam;

     ////dynamci dont fuck with it
  //cam.fovy = 0.53;
  //cam.clip[0] = 0.4135783;
  //cam.clip[1] = 5.560484;
  //cam.window_size[0] = 1560;
  //cam.window_size[1] = 960;
  //cam.window_pos[0] = 65;
  //cam.window_pos[1] = 52;
  //cam.focal[0] = 0.87;
  //cam.focal[1] = 0.002;
  //cam.focal[2] = 0.38;
  //cam.pos[0] = -0.89135;
  //cam.pos[1]= 0.07366;
  //cam.pos[2]= 1.579;
  //cam.view[0] = 0.55;
  //cam.view[1] = 0.029;
  //cam.view[2] = 0.82;

  //viewer->setCameraParameters(cam,1);


  //viewer->setBackgroundColor(1,1,1);
  //ss.str("");
  //ss << folder << "intensity_" << i << ".png";
  //viewer->saveScreenshot(ss.str());

 //while (( std::clock() - start ) / (double)CLOCKS_PER_SEC <0.001)
////  while (!viewer->wasStopped ())
//// while(!close_viewer)
  //{
   //viewer->spinOnce (100);
   //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  //}
////  getchar();
  //viewer->removeAllPointClouds ();
  //viewer->removeAllShapes ();



 }

}
