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



#include "dynamic_filter_node.h"
using namespace Eigen;
using namespace std;
///choose correspondences which minimzes the distrtion. The neighbourhood
//structure is known in the source frame and in a perfect case the neighbourhood
//structure should be similar. The euclidean distance between sampled points is
//calculated and those correspondences are chosen which have similar euclidean
//distance
void DynamicFilter::filter_correspondences(const float neighbour_radius,const float covariance_value,const float score_threshold)
{

 c_vec_actual.clear();
 std::vector< std::pair<int,float> >neighbour_info;
 std::vector <int> query_vec;
 for ( int i = 0 ;i < c_vec.size(); ++i)
 {
  query_vec.push_back(c_vec[i].query);
  std::pair<int,float>element;
  element.first = i;
  element.second = c_vec[i].score[0];
  neighbour_info.push_back(element);
 }
 ///sorting on the basis of correspondence score
 std::sort(neighbour_info.begin(),neighbour_info.end(),boost::bind(&std::pair<int, float>::second, _1) < boost::bind(&std::pair<int, float>::second, _2));

 std::vector <bool> updated_correspondences(c_vec.size(),false);
 std::vector <bool> is_actual(c_vec.size(),false);
 std::vector <int> actual_index;///storing index of filtered correspondence

////Find the best correspondence, inital seed
 for(size_t i = c_vec.size() -1 ; i > c_vec.size() - 3; --i)
 {
  c_vec_actual.push_back(c_vec[neighbour_info[i].first]);
  updated_correspondences[neighbour_info[i].first] = true;
  is_actual[neighbour_info[i].first] = true;
  actual_index.push_back(neighbour_info[i].first);
 }

 PointCloud::Ptr cloud_actual_correspondences(new PointCloud);
 PointCloud::Ptr cloud_correspondences(new PointCloud);
///pointcloud of all the correspondences
 pcl::copyPointCloud(*frame_1.raw_input,query_vec,*cloud_correspondences);


 float first_constant = log(2*M_PI) + log(covariance_value * covariance_value);


 std::vector < std::vector <int> > neighbours;
///These two data structure helps in avoiding calculation of likelihhod twice
//w.r.t to an acctual correspondene

 std::vector < std::vector <int> > correspondence_neighbours(c_vec.size());//point in the neighbourhood which is an actual correspondences
 std::vector < std::vector <bool> > correspondence_visited_neighbours(c_vec.size());///

 pcl::search::KdTree <pcl::PointXYZ> kdtree;
 kdtree.setInputCloud(cloud_correspondences);

//////Find the neighbours for all the point and store them
 for(size_t l = 0; l < c_vec.size(); ++l)
 {
  std::vector <int> pointIdxRadiusSearch;
  std::vector <float> pointRadiusSquaredDistance;
  float normalization = 0.0;
  if (kdtree.radiusSearch(frame_1.raw_input->points[c_vec[l].query],neighbour_radius,pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0)
  {
   neighbours.push_back(pointIdxRadiusSearch);///index of all the possible neighboring correspondences
  }
  for(auto i:actual_index)//index of c_vec that are actual correspondences
  {


   auto low_val =  std::find(pointIdxRadiusSearch.begin(),pointIdxRadiusSearch.end(),i);
   if(low_val!= pointIdxRadiusSearch.end())
   {
    correspondence_neighbours[l].push_back(i);
    correspondence_visited_neighbours[l].push_back(false);
   }

  }
 }
// cout << "neighbours" << neighbours.size() << endl;
 for(size_t i = 0; i < c_vec.size(); ++i)
  c_vec[i].likelihood.assign(c_vec[i].possible_matches.size(),0.0);


 std::vector <correspondences> c_best_vec;
 std::vector <std::pair<int,float> >c_best_sort;
 int count = 0;
////Loop for filtering. For each unfiltered correspondence, look for an actual
//correspondence in its small neighbourhood. If correspondence are avialable use
//it to calculate likelihood of it being it an actual correspondence. Choose the
//best correspondence(likelihood score) and add it to actual correspondence
//list.




 for(size_t l = 0; l < c_vec.size(); ++l)
 {
  c_best_vec.clear();
  c_best_sort.clear();

  c_best_vec.resize(c_vec.size());
  c_best_sort.resize(c_vec.size());

  correspondences c_best;
  c_best.score.assign(1,-1000);
  c_best.possible_matches.assign(1,-1);
  c_best.query = -1;
  c_best.match = -1;
  for(size_t i = 0; i < c_vec.size(); ++i)
  {
   c_vec[i].probability.resize(c_vec[i].possible_matches.size());
   if(is_actual[i])///skip if already filtered
    continue;
   float normalization = 0.0;
   updated_correspondences[i] = true;
   for(size_t k = 0; k < c_vec[i].possible_matches.size(); ++k)
   {
    for(size_t j = 0; j < correspondence_neighbours[i].size(); ++j)
    {
     if(correspondence_visited_neighbours[i][j])///likelihood is not added twice
      continue;
     if(c_vec[i].possible_matches[k] >=frame_2.raw_input->points.size() ||c_vec[correspondence_neighbours[i][j]].match >=frame_2.raw_input->points.size())
     {

      cout << c_vec[i].possible_matches[k] << "," << c_vec[correspondence_neighbours[i][j]].match << "," <<  frame_2.raw_input->points.size() << endl;

      getchar();

     }
     float observed_length = (frame_2.raw_input->points[c_vec[i].possible_matches[k]].getVector3fMap() - frame_2.raw_input->points[c_vec[correspondence_neighbours[i][j]].match].getVector3fMap()).lpNorm<2>();///distance between neighbouring points in target scan
     float actual_length = (frame_1.raw_input->points[c_vec[i].query].getVector3fMap()- frame_1.raw_input->points[c_vec[correspondence_neighbours[i][j]].query].getVector3fMap()).lpNorm<2>();///distance between neighboring points in source scan

     c_vec[i].likelihood[k] += ((observed_length - actual_length)*(observed_length - actual_length));
    }
    c_vec[i].probability[k] = log(c_vec[i].score[k]) +(((-1.0 * correspondence_neighbours[i].size()/2) * first_constant) -((1.0/(2 *covariance_value * covariance_value)) * c_vec[i].likelihood[k]));

///keep track of the best score
    if(c_vec[i].probability[k] > c_best.score[0])
    {
     c_best.score[0] = c_vec[i].probability[k];
     c_best.query = c_vec[i].query;
     c_best.match = c_vec[i].possible_matches[k];
     c_best.possible_matches[0] = i;
    }
   }
   for(size_t j = 0; j < correspondence_neighbours[i].size(); ++j)
    correspondence_visited_neighbours[i][j] = true;//makes sure the likelihood for this point is not added again
  }

  ////if no correspondence have any selected correspondence in its neighbourhood
  //add the best correspondence at that moment
  if(c_best.score[0] == -1000)
   {
    for(int i = c_vec.size() -1 ; i >= 0; i--)
    {
     if(!updated_correspondences[neighbour_info[i].first])
     {
      updated_correspondences[neighbour_info[i].first] = true;

     if(neighbour_info[i].second > score_threshold)
      {
       auto low_val = std::find(neighbours[i].begin(),neighbours[i].end(),neighbour_info[i].first);
       if(low_val!= neighbours[i].end())
       {
        correspondence_neighbours[i].push_back(neighbour_info[i].first);
        correspondence_visited_neighbours[i].push_back(false);
       }
       is_actual[neighbour_info[i].first] = true;
      }
     break;
     }
    }
    continue;
   }
  c_vec_actual.push_back(c_best);
  for(size_t i = 0; i < c_vec.size(); ++i)
  {
   if(is_actual[i])
    continue;
   auto low_val =  std::find(neighbours[i].begin(),neighbours[i].end(),c_best.possible_matches[0]);
   if(low_val!= neighbours[i].end())
   {
    correspondence_neighbours[i].push_back(c_best.possible_matches[0]);
    correspondence_visited_neighbours[i].push_back(false);
   }
  }
  is_actual[c_best.possible_matches[0]] = true;
 }

}
///For each point calculate finite possible correspondences and then filter the
//correspondences by minimizing the distortion caused by the correspondences.
//We assume a maximum motion between two frames, so instead of looking for
//corresponding point in the whole scan, just use the neighbourhood defined by
//maximum neighbourhood.

void DynamicFilter::EstimateCorrespondencePoint(const float radius,const float sampling_radius,int number_correspondences,std::vector<int>&index_query, std::vector<int> &index_match)

{

 c_vec.clear();
 pcl::PointCloud <int> sampled_indices;
 pcl::UniformSampling <pcl::PointXYZ> uniform_sampling;
 uniform_sampling.setInputCloud (frame_1.cloud_input);
 uniform_sampling.setRadiusSearch (sampling_radius);//0.4 in general
 uniform_sampling.compute (sampled_indices);

 for(auto &i:sampled_indices.points)
  frame_1.sampled_points.push_back(frame_1.finite_points[i]);

 PointCloud::Ptr cloud_source(new PointCloud);
 SHOTCloud::Ptr source(new SHOTCloud);
 pcl::copyPointCloud(*frame_1.cloud_input,sampled_indices.points,*cloud_source);
 pcl::copyPointCloud(*frame_1.feature,sampled_indices.points,*source);
 MatrixXf source_feature = source->getMatrixXfMap(352,361,0);
 MatrixXf target_feature = frame_2.feature->getMatrixXfMap(352,361,0);



 pcl::search::KdTree<pcl::SHOT352> kdtree;
 pcl::search::KdTree<pcl::PointXYZ>kdtree_point;
 kdtree_point.setInputCloud (frame_2.cloud_input);

 int counter = 0;
 int counter_check = 0;

 for(auto &point:cloud_source->points)
 {
  std::vector <int> pointIdxRadiusSearch;
  std::vector <float> pointRadiusSquaredDistance;
  if ( kdtree_point.radiusSearch (point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0 )
  {
   counter_check+=1;
   VectorXf s_v = source_feature.col(counter);
   VectorXf score_vec (pointIdxRadiusSearch.size());

   std::vector< std::pair <int,float> >neighbour_info;
   for(int i = 0 ; i < pointIdxRadiusSearch.size() ; ++i)
   {
    VectorXf diff = s_v - target_feature.col(pointIdxRadiusSearch[i]);
    std::pair<int,float>element;
    element.first = frame_2.finite_points[pointIdxRadiusSearch[i]];
    element.second = (diff).dot((diff));
    neighbour_info.push_back(element);
   }

  	std::sort(neighbour_info.begin(),neighbour_info.end(),boost::bind(&std::pair<int, float>::second, _1) < boost::bind(&std::pair<int, float>::second, _2));
   correspondences c;

   c.query = frame_1.sampled_points[counter];

   if(number_correspondences > pointIdxRadiusSearch.size())
    number_correspondences = pointIdxRadiusSearch.size();
   for(int i = 0; i < number_correspondences ; ++i)
   {
    if(neighbour_info[i].second < feature_score_threshold)
    {
     c.possible_matches.push_back(neighbour_info[i].first);
     c.score.push_back( 1.0 - neighbour_info[i].second);
    }
   }
   if(c.score.size() > 0)
   {
    c.match = c.possible_matches[0];
    c_vec.push_back(c);

   }


  }

  counter+=1;

 }
 if(c_vec.empty())
  return;




 filter_correspondences(filter_radius,filter_variance,0.65);


 pcl::PCDWriter writer;

#if 1
 if(!frame_1.cloud_input->points.empty() && !frame_2.cloud_input->points.empty())
 {

// frame_1.ground->width = frame_1.ground->points.size();
// frame_1.ground->height = 1;


// ss.str("");
 //ss << output_folder << "ground_a_" << frame_1.frame_id << ".pcd";
// writer.write(ss.str(),*frame_1.ground,true);

 for(auto &c:c_vec_actual)
  {
   index_query.push_back(c.query);
   index_match.push_back(c.match);

  }
 }
#else
 if(!frame_1.cloud_input->points.empty() && !frame_2.cloud_input->points.empty())
 {
  frame_1.raw_input->width = frame_1.raw_input->points.size();
  frame_1.raw_input->height = 1;

  frame_2.raw_input->width = frame_2.raw_input->points.size();
  frame_2.raw_input->height = 1;

  frame_1.ground->width = frame_1.ground->points.size();
  frame_1.ground->height = 1;

  frame_2.ground->width = frame_2.ground->points.size();
  frame_2.ground->height = 1;


  ss.str("");
  ss << output_folder << "a_" << frame_1.frame_id << ".pcd";
  writer.write(ss.str(),*frame_1.raw_input,true);

  ss.str("");
  ss << output_folder << "ground_a_" << frame_1.frame_id << ".pcd";
  writer.write(ss.str(),*frame_1.ground,true);

  ss.str("");
  ss << output_folder << "a_" << frame_2.frame_id << ".pcd";
  writer.write(ss.str(),*frame_2.raw_input,true);

  ss.str("");
  ss << output_folder << "ground_a_" << frame_2.frame_id << ".pcd";
  writer.write(ss.str(),*frame_2.ground,true);


  ss.str("");
  ss << output_folder << "query_a_" << frame_1.frame_id << ".csv";

  ofstream myfile_query(ss.str().c_str());

  ss.str("");
  ss << output_folder << "match_a_" << frame_1.frame_id << ".csv";


  ofstream myfile_match(ss.str().c_str());
   for(auto &c:c_vec_actual)
  {
   index_query.push_back(c.query);
   index_match.push_back(c.match);

   myfile_query << c.query << endl;
   myfile_match << c.match << endl;
  }
 /*
  for(auto &query:index_query)
   myfile_query << query << endl;

  for(auto &match:index_match)
   myfile_match << match << endl;
   ///
  myfile_query.close();
  myfile_match.close();

*/


 }
#endif



}

///Estimate correspondences for static points using nearest neighbours instead
//of using features.


void DynamicFilter::EstimateCorrespondenceEuclidean(const float sampling_radius,std::vector<int>&index_query, std::vector<int> &index_match,std::vector<int> &indices_dynamic)
{
///Associtaing points in scan t-1(frame_1.cloud_transformed) and points in scan
//t(frame_1.raw_input)
 pcl::Correspondences corr;
 pcl::registration::CorrespondenceEstimation <Point,Point> est;
 est.setInputSource (frame_1.raw_input);
 est.setInputTarget (frame_1.cloud_transformed);
 est.determineCorrespondences (corr);
 const double max_distance = 0.01 * 0.01;
 std::vector <int> indices_corr;
 PointCloud::Ptr cloud_trans(new PointCloud);
 for(auto &corr_point:corr)
 {
  if(corr_point.distance < (max_distance))//actual correspondence
  {
   if(frame_1.prior_dynamic[corr_point.index_match] < 0.15)//static Point
   {
    Vector4f point = frame_1.raw_input->points[corr_point.index_query].getVector4fMap();
    point[3] = 1;
    Vector4d point_trans = frame_1.motion_init[corr_point.index_match] * point.cast<double>();//Predicitng the location of point in scan t to location of scan t+1 using the motion calculated between t-1 and t
    Point pointPCL;
    pointPCL.x = point_trans[0];
    pointPCL.y = point_trans[1];
    pointPCL.z = point_trans[2];
    indices_corr.push_back(corr_point.index_query);
    cloud_trans->points.push_back(pointPCL);
   }
  }
 }

///uniform sampling for correspondences
 pcl::PointCloud <int> sampled_indices;
 pcl::UniformSampling <pcl::PointXYZ> uniform_sampling;
 uniform_sampling.setInputCloud (cloud_trans);
 uniform_sampling.setRadiusSearch (sampling_radius);//0.4 in general
 uniform_sampling.compute (sampled_indices);

 PointCloud::Ptr cloud_trans_sampled(new PointCloud);

 pcl::copyPointCloud(*cloud_trans,sampled_indices.points, *cloud_trans_sampled);
 ///calculating correspondence between t and t+1
 pcl::Correspondences corr_final;
 est.setInputSource (cloud_trans_sampled);
 est.setInputTarget (frame_2.raw_input);
 est.determineReciprocalCorrespondences (corr_final);

///segmenting the static scene from the whole scan

 pcl::search::KdTree <pcl::PointXYZ> kdtree;
 kdtree.setInputCloud(frame_2.raw_input);

//////Find the neighbours for all the point and store them
 std::vector <bool>is_dynamic(frame_2.raw_input->points.size(),true);
 for(auto &c:corr_final)
 {

  index_query.push_back(indices_corr[sampled_indices.points[c.index_query]]);
  index_match.push_back(c.index_match);
  std::vector <int> pointIdxRadiusSearch;
  std::vector <float> pointRadiusSquaredDistance;
  if (kdtree.radiusSearch(frame_2.raw_input->points[c.index_match],0.05,pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0)
  {
   for(auto &index:pointIdxRadiusSearch)
   is_dynamic[index] = false;
  }
 }
 std::vector<int>frame_2_dynamic;
 for(size_t i = 0; i < is_dynamic.size(); ++i)
 {
  if(is_dynamic[i])
   indices_dynamic.push_back(i);
 }

}





