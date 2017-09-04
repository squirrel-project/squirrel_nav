// The MIT License (MIT)
//
// Copyright (c) 2016-2017 Ayush Dewan and Wolfram Burgard
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
#include "datatypes_squirrel.h"
#include <pcl/visualization/pcl_visualizer.h>

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
 int start_frame = atoi(argv[1]);
 int end_frame = atoi(argv[2]);

 string folder = argv[3];

 pcl::PCDReader reader;

 IntensityCloud::Ptr cloud(new IntensityCloud);
 IntensityCloud ground;
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


 for(size_t i = start_frame; i < end_frame; ++i)
 {
  cloud->points.clear();
  cout << i << endl;
  ss.str("");
  ss << folder << "cloud_save_robust_a_" << i << ".pcd";

  reader.read(ss.str(),*cloud);
  ss.str("");
  ss << folder << "ground_" << i << ".pcd";

  reader.read(ss.str(),ground);

  ss.str("");
  ss << folder << "intensity_z_" << i << ".csv";

  myfile.open(ss.str().c_str());
  int counter = 0;
  if (myfile.is_open())
  {
   while ( getline (myfile,line) )
   {
  //  if(atof(line.c_str()) < 0.02)
     cloud->points[counter].intensity = atof(line.c_str());
  //  else
  //   cloud->points[counter].intensity = 0.1;

    counter +=1;
   }
  }

  for(auto &point:ground.points)
  {
   point.intensity = 0.0;
   cloud->points.push_back(point);
  }
  myfile.close();
  pcl::PointXYZI min,max;
  cout << cloud->points.size() << endl;
  min.x = max.x = 0.0;
  min.y = max.y = 0.0;
  min.z = max.z = 0.0;
  min.intensity = 0;
  max.intensity = 1.0;
//  Eigen::Vector3f trans(motion_trans[i][0],motion_trans[i][1],motion_trans[i][2]);

//  cout << trans.lpNorm<2>() << "," << 2 * acos(motion_trans[i][6]) << endl;
  cloud->points.push_back(min);
  cloud->points.push_back(max);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgb(cloud,"intensity");
  viewer->addPointCloud<pcl::PointXYZI> (cloud,rgb,"cloud");
  std::clock_t start;
  double duration;
  start = std::clock();
  close_viewer = false;
  ss.str("");
  ss << folder << "intensity_" << i << ".png";
  viewer->saveScreenshot(ss.str());
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
 pcl::visualization::Camera cam;

     //dynamci dont fuck with it
    cam.fovy = 0.53;
    cam.clip[0] = 0.0235783;
    cam.clip[1] = 20.90484;
    cam.window_size[0] = 1560;
    cam.window_size[1] = 960;
    cam.window_pos[0] = 65;
    cam.window_pos[1] = 52;
    cam.focal[0] = 0.653;
    cam.focal[1] = 0.17;
    cam.focal[2] = 0.64;
    cam.pos[0] = -2.24135;
    cam.pos[1]= -0.217366;
    cam.pos[2]= 3.1731584;
    cam.view[0] = 0.64936;
    cam.view[1] = 0.059689;
    cam.view[2] = 0.75;


   viewer->setCameraParameters(cam,1);


    viewer->setBackgroundColor(1,1,1);

// while (( std::clock() - start ) / (double)CLOCKS_PER_SEC <0.05)
 // while (!viewer->wasStopped ())
 while(!close_viewer)
  {
   viewer->spinOnce (100);
   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
//  getchar();
  viewer->removeAllPointClouds ();
  viewer->removeAllShapes ();



 }

}
