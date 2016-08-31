#include "dynamic_filter_node.h"
using namespace std;

DynamicFilter::DynamicFilter():cloud_input(new PointCloud),cloud_input_objects(new PointCloud),tf_()
{

 cloud_sub_1 = n_.subscribe("/squirrel/cloud_msg", 5000, &DynamicFilter::cloudMsgCallback, this);
  Vector7d sensor_to_base_link;
////transformation between sensor and base_link, required to remove the flat
//ground" 
#if 0
 sensor_to_base_link(0) = 0.049;
 sensor_to_base_link(1) = 0.023;
 sensor_to_base_link(2) = 0.740;
 sensor_to_base_link(3) = 0.685;
 sensor_to_base_link(4) = -0.642;
 sensor_to_base_link(5) = 0.235;
 sensor_to_base_link(6) = -0.250;
#else
 sensor_to_base_link(0) = 0.056;
 sensor_to_base_link(1) = 0.023;
 sensor_to_base_link(2) = 0.240;
 sensor_to_base_link(3) = 0.618;
 sensor_to_base_link(4) = -0.581;
 sensor_to_base_link(5) = 0.362;
 sensor_to_base_link(6) = -0.386;
#endif

 Isometry3D sensor_to_base_link_trans = g2o::internal::fromVectorQT(sensor_to_base_link);
 trans(0,0) = sensor_to_base_link_trans(0,0);
 trans(0,1) = sensor_to_base_link_trans(0,1);
 trans(0,2) = sensor_to_base_link_trans(0,2);
 trans(0,3) = sensor_to_base_link_trans(0,3);
 trans(1,0) = sensor_to_base_link_trans(1,0);
 trans(1,1) = sensor_to_base_link_trans(1,1);
 trans(1,2) = sensor_to_base_link_trans(1,2);
 trans(1,3) = sensor_to_base_link_trans(1,3);
 trans(2,0) = sensor_to_base_link_trans(2,0);
 trans(2,1) = sensor_to_base_link_trans(2,1);
 trans(2,2) = sensor_to_base_link_trans(2,2);
 trans(2,3) = sensor_to_base_link_trans(2,3);
 trans(3,0) = sensor_to_base_link_trans(3,0);
 trans(3,1) = sensor_to_base_link_trans(3,1);
 trans(3,2) = sensor_to_base_link_trans(3,2);
 trans(3,3) = sensor_to_base_link_trans(3,3);
 pub_tracked_gt = n_.advertise<sensor_msgs::PointCloud2>("/cloud/busstop_gt",1000);

 is_first_frame = false;
 frame_counter = 0;
 n_.getParam("/DownSamplingRadius",down_sampling_radius);
 n_.getParam("/FeatureClusterMaxLength",feature_cluster_max_length);
 n_.getParam("/FeatureClusterMaxPoints",feature_cluster_max_points);
 n_.getParam("/FeatureRadius",feature_radius);
 n_.getParam("/FeatureScoreThreshold",feature_score_threshold);
 n_.getParam("/FilterRadius",filter_radius);
 n_.getParam("/FilterVariance",filter_variance);
 n_.getParam("/KeyPointRadius",keypoint_radius);
 n_.getParam("/NormalRadius",normal_radius);
 n_.getParam("/OutputFolder",output_folder);
 n_.getParam("/MaxMotion",max_motion);
 n_.getParam("/SamplingRadius",sampling_radius);

 ss.str("");
 ss << output_folder << "parameteres.csv";
 ofstream myfile_params(ss.str().c_str());

 myfile_params << down_sampling_radius << endl << feature_cluster_max_length << endl << feature_cluster_max_points << endl << feature_radius << endl << feature_score_threshold << endl << filter_radius << endl << filter_variance << endl << keypoint_radius << endl << normal_radius << endl << max_motion << endl << sampling_radius;

 myfile_params.close();

 ss.str("");
 ss << output_folder << "odometry.csv";
 myfile_odom.open(ss.str().c_str());

 ss.str("");
 ss << output_folder << "time.csv";
 time_write.open(ss.str().c_str());







}



//////Callback for the message. Each message contains a pointcloud and odometry 
void DynamicFilter::cloudMsgCallback(const squirrel_dynamic_filter_msgs::CloudMsg& sensor_msg) 
{

 sensor_msgs::PointCloud2 cloud_input_object_msg;
/////If its first frame do nothing just fill up the data
 if(!is_first_frame)
 {
  pcl::fromROSMsg(sensor_msg.cloud_msg,*frame_1.raw_input);
  Vector7d odometry;
  odometry[0] = sensor_msg.odometry[0];
  odometry[1] = sensor_msg.odometry[1];
  odometry[2] = sensor_msg.odometry[2];
  odometry[3] = sensor_msg.odometry[3];
  odometry[4] = sensor_msg.odometry[4];
  odometry[5] = sensor_msg.odometry[5];
  odometry[6] = sensor_msg.odometry[6];

  myfile_odom << odometry[0] << "," << odometry[1] << "," << odometry[2] << "," << 
odometry[3] << "," << odometry[4] << "," << odometry[5] << "," << odometry[6] << endl;

  frame_1.odometry = g2o::internal::fromVectorQT(odometry); 	
  frame_1.frame_id = frame_counter;
  preprocessing(frame_1);///remove ground
  EstimateFeature(frame_1);
  is_first_frame = true;
 }
 else/////Second frame fill up the data amd start running the stuff
 { 
  start_total = SystemClock::now();
  frame_2.clear();
  pcl::fromROSMsg(sensor_msg.cloud_msg,*frame_2.raw_input);
  Vector7d odometry;
  odometry[0] = sensor_msg.odometry[0];
  odometry[1] = sensor_msg.odometry[1];
  odometry[2] = sensor_msg.odometry[2];
  odometry[3] = sensor_msg.odometry[3];
  odometry[4] = sensor_msg.odometry[4];
  odometry[5] = sensor_msg.odometry[5];
  odometry[6] = sensor_msg.odometry[6];
  myfile_odom << odometry[0] << "," << odometry[1] << "," << odometry[2] << "," << 
odometry[3] << "," << odometry[4] << "," << odometry[5] << "," << odometry[6] << endl;
  frame_2.odometry = g2o::internal::fromVectorQT(odometry); 	
  odometry_diff = frame_2.odometry.inverse() * frame_1.odometry; 
  if(frame_2.raw_input->points.size() > 0)
   preprocessing(frame_2);///remove ground
  frame_2.frame_id = frame_counter;
  if(frame_2.raw_input->points.size() > 50)
  {
   start = SystemClock::now(); 
   EstimateFeature(frame_2);////Estimate features
   end = SystemClock::now();
   time_diff = end - start;
   feature_time = time_diff.count();


   std::vector <int> index_query;
   std::vector <int> index_match;
   start = SystemClock::now(); 
   EstimateCorrespondencePoint(max_motion,sampling_radius,5,index_query,index_match);///Estimate correspondences
   end = SystemClock::now();
   time_diff = end - start;
   correspondence_time = time_diff.count();
   start = SystemClock::now(); 
   if(!index_query.empty())
    EstimateMotion(index_query,index_match);///Estimate the motion
   end = SystemClock::now();
   time_diff = end - start;
   motion_time = time_diff.count();
  }

   frame_2.copy(frame_1);
   end_total = SystemClock::now();
   time_diff = end_total - start_total;
   total_time = time_diff.count();

   time_write << feature_time << "," << correspondence_time << "," << motion_time << "," << total_time << endl;


  //cloud_input_object_msg.header.frame_id = "/base_link";
//  pub_tracked_gt.publish(cloud_input_object_msg);

 }
 
 frame_counter+=1;




}

void DynamicFilter::preprocessing(Frame &frame)
{

 PointCloud cloud_nan;
 PointCloud cloud_trans;


 for (auto &point:frame.raw_input->points)
		{
			if(pcl_isfinite(point.x))
					cloud_nan.points.push_back(point);
  }///removes infinite points
  
 pcl::transformPointCloud(cloud_nan,cloud_trans,trans);
 std::vector <bool> is_ground(cloud_trans.points.size(),false);
 std::vector <int> indices;
 std::vector <int> indices_ground;
#pragma omp parallel for
 for(size_t i = 0; i < cloud_trans.points.size();++i)
 {
  if(cloud_trans.points[i].z< 0.02)
   is_ground[i] = true;
 }
 int count = 0;
 for(auto ground:is_ground)
 {
  if(!ground)
   indices.push_back(count);
  else
   indices_ground.push_back(count);

  count+=1;
 }

 PointCloud::Ptr cloud_ground(new PointCloud);
 pcl::copyPointCloud(cloud_trans,indices,*cloud_ground);
 pcl::copyPointCloud(cloud_trans,indices_ground,*frame.ground);
 frame.raw_input->points.clear();
 sor.setInputCloud (cloud_ground);
 sor.setLeafSize (down_sampling_radius,down_sampling_radius,down_sampling_radius);
 sor.filter (*frame.raw_input);
 frame.raw_input->width = frame.raw_input->points.size();
 frame.raw_input->height = 1;
}

int main(int argc,char **argv)
{
 ros::init(argc, argv, "pose_drawer");
 DynamicFilter filter;
	while(ros::ok())
	{
		ros::spinOnce();
 }

}
