#include "datatypes_squirrel.h"
#include <pcl/visualization/pcl_visualizer.h>
#include "edge_unary.h"
#include "edge.h"
#include <mlpack/core.hpp>
#include <mlpack/core/dists/gaussian_distribution.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/impl/correspondence_estimation.hpp>


using namespace std;
using namespace Eigen;
using namespace g2o;

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


int main(int argc,char **argv)
{
 arma::mat covariance(3,3);
 arma::vec mean(3);
 arma::vec observation(3);
 covariance(0,0) = 1.0; covariance(0,1) = 0.0; covariance(0,2) = 0.0;
 covariance(1,0) = 0.0; covariance(1,1) = 1.0; covariance(1,2) = 0.0;
 covariance(2,0) = 0.0; covariance(2,1) = 0.0; covariance(2,2) = 1.0;
 covariance *= 0.1;

 std::vector <float> prior_dynamic;



 int start_frame = atoi(argv[1]);
 int end_frame = atoi(argv[2]);

 string folder = argv[3];

 pcl::PCDReader reader;

 PointCloud::Ptr cloud(new PointCloud);
 PointCloud::Ptr cloud_trans(new PointCloud);
 IntensityCloud ground;
 std::stringstream ss;

 ifstream myfile;
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	std::vector < Vector7d > motion_trans;

 ss.str("");

 ss << folder << "/odometry.csv";
 string line;
	myfile.open(ss.str());
	if (myfile.is_open())
 {
  while ( getline (myfile,line) )
			{

    Vector7d trans;
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
 
 Isometry3D frame_1 = ::g2o::internal::fromVectorQT(motion_trans[start_frame]);
 Isometry3D frame_2 = ::g2o::internal::fromVectorQT(motion_trans[start_frame + 1]);
 Isometry3D odometry = (frame_2.inverse() * frame_1);////odometry from the robot
 mean[0] = odometry(0,3);
 mean[1] = odometry(1,3);
 mean[2] = odometry(2,3);

 observation[0] = odometry(0,3);
 observation[1] = odometry(1,3);
 observation[2] = odometry(2,3);

 mlpack::distribution::GaussianDistribution dist_max(mean,covariance);
 float max = dist_max.Probability(observation);

 ss.str("");
 ss << folder << "cloud_save_robust_a_" << start_frame << ".pcd";

 reader.read(ss.str(),*cloud);

 prior_dynamic.assign(cloud->points.size(),0.2);
 
 


 mlpack::distribution::GaussianDistribution dist(mean,covariance);


 const float prior_static = 0.8;

 const float p_s_d = 0.05;
 const float p_s_s = 0.95;
 const float p_d_s = 0.05;
 const float p_d_d = 0.95;


 int counter = 0;
 ss.str("");
 ss << folder << "motion_a_" << start_frame << ".csv";


 myfile.open(ss.str().c_str());
 if (myfile.is_open())
 {
  std::vector <float> current_belief;
  while ( getline (myfile,line) )
  {
   Vector7d trans;
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
  
   Isometry3D estimated_motion = ::g2o::internal::fromVectorQT(trans);

   Vector4f point = cloud->points[counter].getVector4fMap();

   point[3] = 1.0;

   Vector4d point_trans = estimated_motion * point.cast<double>();

   pcl::PointXYZ PCLpoint;
   PCLpoint.x = point_trans[0];
   PCLpoint.y = point_trans[1];
   PCLpoint.z = point_trans[2];

   cloud_trans->points.push_back(PCLpoint);

   

   Isometry3D motion_diff = estimated_motion.inverse() * odometry;
   
   Vector3d trans_motion(motion_diff(0,3),motion_diff(1,3),motion_diff(2,3));
   observation[0] = motion_diff(0,3);
   observation[1] = motion_diff(1,3);
   observation[2] = motion_diff(2,3);

   float posterior_d = (1.0 - dist.Probability(observation)/max)  * ((p_d_d * prior_dynamic[counter]) + (p_d_s * (1.0 - prior_dynamic[counter])));
   float posterior_s = (dist.Probability(observation)/max) * ((p_s_s * (1.0 - prior_dynamic[counter])) + (p_s_d * prior_dynamic[counter]));

   float probability = posterior_d/(posterior_d + posterior_s); 
   current_belief.push_back(probability);



  








   

//   cout << trans_motion.lpNorm<2>() << endl;
//   cout << dist.Probability(observation)/max << "," << probability << endl;


//   getchar();
   counter +=1;
  }

  prior_dynamic.clear();

  prior_dynamic = current_belief;

  myfile.close();

 }
 for(size_t i = start_frame+1; i < end_frame; ++i)
 {
  cout << i << endl; 
  frame_1 = ::g2o::internal::fromVectorQT(motion_trans[i]);
  frame_2 = ::g2o::internal::fromVectorQT(motion_trans[i + 1]);
  odometry = (frame_2.inverse() * frame_1);////odometry from the robot

  cloud->points.clear();
  ss.str("");
  ss << folder << "cloud_save_robust_a_" << i << ".pcd";
  reader.read(ss.str(),*cloud);

  IntensityCloud::Ptr cloud_intensity(new IntensityCloud);
  
  reader.read(ss.str(),*cloud_intensity);
  ss.str("");

  ss << folder << "ground_a_" << i << ".pcd";
  reader.read(ss.str(),ground);

  pcl::Correspondences all_correspondences;
  pcl::registration::CorrespondenceEstimation<Point, Point> est;
  est.setInputSource (cloud);
  est.setInputTarget (cloud_trans);
  est.determineCorrespondences (all_correspondences);

  
  cloud_trans->points.clear();
  counter = 0;
  ss.str("");
  ss << folder << "motion_a_" << i << ".csv";


  myfile.open(ss.str().c_str());
  if (myfile.is_open())
  {
   std::vector <float> current_belief;
   while ( getline (myfile,line) )
   {
    Vector7d trans;
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
   
    Isometry3D estimated_motion = ::g2o::internal::fromVectorQT(trans);

    Vector4f point = cloud->points[counter].getVector4fMap();

    point[3] = 1.0;

    Vector4d point_trans = estimated_motion * point.cast<double>();

    pcl::PointXYZ PCLpoint;
    PCLpoint.x = point_trans[0];
    PCLpoint.y = point_trans[1];
    PCLpoint.z = point_trans[2];

    cloud_trans->points.push_back(PCLpoint);
    Isometry3D motion_diff = estimated_motion.inverse() * odometry;
    
    Vector3d trans_motion(motion_diff(0,3),motion_diff(1,3),motion_diff(2,3));
    observation[0] = motion_diff(0,3);
    observation[1] = motion_diff(1,3);
    observation[2] = motion_diff(2,3);


    float prior;

    if(all_correspondences[counter].distance <= 0.1)
     prior = prior_dynamic[all_correspondences[counter].index_match];
    else
     prior = 0.2;



    float posterior_d = (1.0 - dist.Probability(observation)/max)  * ((p_d_d * prior) + (p_d_s * (1.0 - prior)));

    float posterior_s = (dist.Probability(observation)/max) * ((p_s_s * (1.0 - prior)) + (p_s_d * prior));


    float probability = posterior_d/(posterior_d + posterior_s); 
    current_belief.push_back(probability);
    //current_belief.push_back(posterior_d);
    if(counter > cloud_intensity->points.size())
    {
     cout << counter << "," << cloud_intensity->points.size() << endl;
     getchar();

    }
    cloud_intensity->points[counter].intensity = probability;
   /*
    if(i == 59)
    {
    cout << trans_motion.lpNorm<2>() << endl;
    cout << dist.Probability(observation)/max << "," << probability << "," << prior << "," << (p_d_d * prior_dynamic[all_correspondences[counter].index_match]) + (p_d_s * (1.0 - prior_dynamic[all_correspondences[counter].index_match])) << "," << ((p_s_s * (1.0 - prior_dynamic[all_correspondences[counter].index_match])) + (p_s_d * prior_dynamic[all_correspondences[counter].index_match])) << "," << posterior_d << "," << posterior_s << endl;
    getchar();
    }
    */
    counter +=1;
   }
   prior_dynamic.clear();

   prior_dynamic = current_belief;

   myfile.close();
  }
  
  for(auto &point:ground.points)
  {
   point.intensity = 0.0;
   cloud_intensity->points.push_back(point);
  }

  pcl::PointXYZI min,max;
  min.x = max.x = 0.0;
  min.y = max.y = 0.0;
  min.z = max.z = 0.0;
  min.intensity = 0;
  max.intensity = 1.0;

  cloud_intensity->points.push_back(min);
  cloud_intensity->points.push_back(max);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rgb(cloud_intensity,"intensity");
  viewer->addPointCloud<pcl::PointXYZI> (cloud_intensity,rgb,"cloud");

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  pcl::visualization::Camera cam;

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
 std::clock_t start;
 double duration;
 start = std::clock(); 

 close_viewer = false;
 ss.str("");
// ss << folder << "p_intensity_" << i << ".png";
// viewer->saveScreenshot(ss.str());
  
  while (( std::clock() - start ) / (double)CLOCKS_PER_SEC <0.001) 
 // while (!viewer->wasStopped ())
//  while(!close_viewer)
  {
   viewer->spinOnce (100);
   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
//  getchar();
  viewer->removeAllPointClouds ();
  viewer->removeAllShapes ();


 } 



return 1;

}
