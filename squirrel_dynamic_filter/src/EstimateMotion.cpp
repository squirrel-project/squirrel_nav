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



////Motion for each cluster is calculated separately. Clustering makes things
//faster and constraints the problem better
void DynamicFilter::EstimateMotion(const std::vector <int> &index_query,const std::vector <int> &index_match,PointCloud &cloud_dynamic)
{
  PointCloud cloud_t_current;
  Isometry3D Identity;
  Identity.setIdentity();
  std::vector <Isometry3D> motion_init_current(frame_1.raw_input->points.size(),Identity);
////

  frame_1.clusters.clear();
  frame_1.neighbours.clear();
  EuclideanClusteringMesh EstimateCluster;
  EstimateCluster.extract_clusters(frame_1.raw_input,frame_1.clusters,frame_1.neighbours,0.3,100,20);

  measure_time start_time = SystemClock::now();

#pragma omp parallel for schedule(dynamic) num_threads(8)

  for( size_t i = 0; i < frame_1.clusters.size(); ++i)
  {
    std::vector<int>cluster = frame_1.clusters[i];
    std::vector <int> index_query_filter;
    std::vector <int> index_match_filter;
    pcl::CorrespondencesPtr initial ( new pcl::Correspondences);
    int counter = 0;


/////Correspondence for each cluster

    for( size_t k = 0; k < cluster.size() ; ++k)
    {
      for( size_t j = 0; j < index_query.size() ; ++j)
      {
        if(cluster[k] == index_query[j])
        {
          pcl::Correspondence corr;
          corr.index_query = counter;
          corr.index_match = index_match[j];
          initial->push_back(corr);
					index_query_filter.push_back(k);
          index_match_filter.push_back(index_match[j]);
          counter +=1;
          break;
        }
      }
    }
  ////if they are less than 3 keypoints, do not opitmize

    if(index_query_filter.size() < 3)
      continue;
    PointCloud::Ptr kp (new PointCloud);
    PointCloud::Ptr cloud_cluster (new PointCloud);
    PointCloud::Ptr cloud_cluster_d_mean (new PointCloud);
///From original cloud copy the cluster and keypoints in the cluster

    pcl::copyPointCloud(*frame_1.raw_input, cluster, *cloud_cluster);

    pcl::copyPointCloud(*cloud_cluster,index_query_filter,*kp);
/*
  pcl::copyPointCloud(*frame_1.cloud_input, cluster, *cloud_cluster_d_mean);


  Eigen::Matrix< float, 4, 1 > centroid;
  pcl::compute3DCentroid<pcl::PointXYZ>(*cloud_cluster_d_mean, centroid);

  pcl::demeanPointCloud(*cloud_cluster_d_mean,centroid,*cloud_cluster);


*/


    BlockSolverX::LinearSolverType * linearSolver(new LinearSolverCSparse<BlockSolverX::PoseMatrixType>());
    BlockSolverX* blockSolver(new BlockSolverX(linearSolver));
    OptimizationAlgorithmLevenberg* optimizationAlgorithm(new OptimizationAlgorithmLevenberg(blockSolver));
    SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    optimizer.setAlgorithm(optimizationAlgorithm);

#if 0
////Initialize the vertextes of the graph. If the first frame each vertex's
//initial value is identity otherwise the vertex's initial value is the
//previoulsy estimated motion
///If first frame
  if(frame_1.motion_init.empty())
			{
    for(size_t k = 0; k < cloud_cluster->points.size() ; ++k)
    {
     Vector3D position_input;
     Vector3f pos = cloud_cluster->points[k].getVector3fMap();
     position_input[0]=pos[0];
     position_input[1]=pos[1];
     position_input[2]=pos[2];
     VertexSE3_Vector3D* vertex_test= new VertexSE3_Vector3D;
     vertex_test->setId(k);
     vertex_test->setToOriginImpl();
     vertex_test->setPosition(position_input);
     optimizer.addVertex(vertex_test);
    }
   }
  else//// if not previous frame
  {
// cout << "temporal" <<endl;
   pcl::Correspondences all_correspondences;
   pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
   est.setInputSource (cloud_cluster);
   est.setInputTarget (frame_1.cloud_transformed);
   est.determineReciprocalCorrespondences (all_correspondences);
   std::vector <bool> is_correspondence ( cloud_cluster->points.size() , false);
   std::vector <int> match ( cloud_cluster->points.size() , -1);
   for ( size_t j = 0; j < all_correspondences.size(); ++j)
   {
    is_correspondence[all_correspondences[j].index_query]=true;
    match[all_correspondences[j].index_query] = all_correspondences[j].index_match;
   }
   for(size_t k = 0; k < cloud_cluster->points.size() ; ++k)
   {
    Vector3D position_input;
    Vector3f pos=cloud_cluster->points[k].getVector3fMap();
    position_input[0]=pos[0];
    position_input[1]=pos[1];
    position_input[2]=pos[2];
    VertexSE3_Vector3D* vertex_test= new VertexSE3_Vector3D;
    vertex_test->setId(k);
    if(is_correspondence[k])
      vertex_test->setEstimate(frame_1.motion_init[match[k]]);
    else
      vertex_test->setToOriginImpl();
    vertex_test->setPosition(position_input);
    optimizer.addVertex(vertex_test);
   }
  }

#else
  ////Initialize each vertex with the robot's odometry instead of uisng previous
  //motion
  for(size_t k = 0; k < cloud_cluster->points.size() ; ++k)
  {
   Vector3D position_input;
   Vector3f pos = cloud_cluster->points[k].getVector3fMap();
   position_input[0]=pos[0];
   position_input[1]=pos[1];
   position_input[2]=pos[2];
   VertexSE3_Vector3D* vertex_test= new VertexSE3_Vector3D;
   vertex_test->setId(k);
   vertex_test->setEstimate(odometry_diff);
   vertex_test->setPosition(position_input);
   optimizer.addVertex(vertex_test);
  }
#endif
  ///Initializing unary edges connecting correspondences

  for(size_t k = 0;  k < index_match_filter.size(); ++k)
  {
    Vector3D position_input;
    Vector3f pos = frame_2.raw_input->points[index_match_filter[k]].getVector3fMap();
    Vector3f pos_1 = cloud_cluster->points[index_query_filter[k]].getVector3fMap();
    position_input[0] = pos[0];
    position_input[1] = pos[1];
    position_input[2] = pos[2];
    EdgeICP* correspondence=new EdgeICP;
    correspondence->vertices()[0] = optimizer.vertex(index_query_filter[k]);
    correspondence->setMeasurement(position_input);
    MatrixXd information=MatrixXd::Identity(3,3);
    correspondence->setInformation(information);
    optimizer.addEdge(correspondence);
  }
  MatrixXint edge_matrix = MatrixXint::Zero(cloud_cluster->points.size(),cloud_cluster->points.size());

// Connecting neighboring points for binaey edges

  for ( size_t k = 0; k < cluster.size() ; ++k)
  {
    int check_neighbours = 0;
    for( size_t j = 0; j < frame_1.neighbours[cluster[k]].size() ; ++j)
    {
      std::vector<int>::iterator it = std::find(cluster.begin() , cluster.end(), frame_1.neighbours[cluster[k]][j]);
      int pos =  it - cluster.begin();
      if(pos > cluster.size())
      {
        cout << pos << "," << cluster.size() << endl;
        getchar();
      }
      if(edge_matrix(k,pos) == 0)
      {
        check_neighbours+=1;
		    MatrixXd information=MatrixXd::Identity(6,6);
		    double weight=(cloud_cluster->points[k].getVector3fMap()-cloud_cluster->points[pos].getVector3fMap()).lpNorm<2>();
		    if(weight==0)
		    {

		      continue;

		      cout<<"found_error"<<endl;

		    }
        Edge* odometry = new Edge;
		    odometry->vertices()[0] = optimizer.vertex(k);
		    odometry->vertices()[1] = optimizer.vertex(pos);
		    odometry->setMeasurement(Isometry3D::Identity());
		    odometry->setInformation(information);
		    optimizer.addEdge(odometry);
		    edge_matrix(k,pos)=edge_matrix(pos,k)=1;
     }
    }

   }

   optimizer.initializeOptimization();
   SparseOptimizerTerminateAction* terminateAction = 0;
   terminateAction = new SparseOptimizerTerminateAction;
   terminateAction->setGainThreshold(0.08);
   terminateAction->setMaxIterations(5);
   optimizer.addPostIterationAction(terminateAction);
   optimizer.optimize(5);
////Second round of optimzation
   g2o::HyperGraph::VertexIDMap ver = optimizer.vertices();
   g2o::HyperGraph::EdgeSet edge_set = optimizer.edges();
   ver = optimizer.vertices();
   for(auto it=ver.begin();it!=ver.end();++it)
   {
    g2o::OptimizableGraph::Vertex* v=static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
    std::vector <double> data;
    v->getMinimalEstimateData(data);
    Vector6d data_new;

    data_new(0) = data[0];data_new(1) = data[1];data_new(2) = data[2];data_new(3) = data[3];data_new(4) = data[4];data_new(5) = data[5];
  //  cout << data_new.lpNorm<2>() << endl;;
    Isometry3D motion = g2o::internal::fromVectorMQT(data_new);
//    cout << cluster[v->id()] << "," << motion_init_current.size() << endl;
    motion_init_current[cluster[v->id()]] = motion;
   }

 }

  PointCloud::Ptr score(new PointCloud);///used for estimaing score of static points
  score->points = frame_1.cloud_transformed->points;
  frame_1.cloud_transformed->points.clear();
  frame_1.motion_init.clear();
  PointCloud::Ptr cloud_save(new PointCloud);
  ofstream write_trans;
  ofstream write_intensity_z;
  if(store_results)
  {
    ss.str("");
    ss << output_folder << "motion_a" << "_" << frame_1.frame_id << ".csv";
    write_trans.open(ss.str().c_str());
    ss.str("");
    ss << output_folder << "intensity_z" << "_" << frame_1.frame_id << ".csv";
    write_intensity_z.open(ss.str().c_str());

  }
 int counter = 0;

 for (auto &PCLpoint:frame_1.raw_input->points)
 {
   Vector4f point = PCLpoint.getVector4fMap();
   point(3) = 1.0;
   Vector4d point_final = motion_init_current[counter] * point.cast<double>();
   Vector6d motion_compact = g2o::internal::toVectorMQT(motion_init_current[counter] );
   if(motion_compact.lpNorm<2>()!=0)
   {
     Isometry3D motion_diff = motion_init_current[counter].inverse() * odometry_diff;

     Eigen::Vector2d trans(motion_diff(0,3),motion_diff(1,3));

     //write_intensity << trans.lpNorm<2>() << endl;

     Eigen::Vector3d trans_z(motion_diff(0,3),motion_diff(1,3),motion_diff(2,3));
     Vector7d motion = g2o::internal::toVectorQT(motion_init_current[counter] );
     frame_1.motion_init.push_back(motion_init_current[counter]);
     pcl::PointXYZ point_cloud;
     point_cloud.x = point_final[0];
     point_cloud.y = point_final[1];
     point_cloud.z = point_final[2];
     frame_1.cloud_transformed->points.push_back(point_cloud);

     point_cloud.x = point[0];
     point_cloud.y = point[1];
     point_cloud.z = point[2];
     cloud_save->points.push_back(point_cloud);
     if(store_results)
       write_trans << motion(0) << "," << motion(1) << "," <<motion(2) << "," << motion(3) << "," << motion(4) << "," << motion(5) << "," << motion[6] << endl;
   }
   counter +=1;
 }
 if(cloud_save->points.size() > 0 && score->points.size() >0 )
 {
   bool check = false;
   if(frame_1.prior_dynamic.empty())
   check = true;
   DynamicScore(cloud_save,check,score); ///calculate score for each point

   int count_point = 0;

   for(auto &score:frame_1.prior_dynamic)
   {
     if(score < 0.05)///if point is static
       cloud_dynamic.points.push_back(cloud_save->points[count_point]);
     if(store_results)
       write_intensity_z << score << endl;
     count_point += 1;
   }
 }
 else
   frame_1.prior_dynamic.clear();

 if(store_results)
 {
   write_trans.close();
   write_intensity_z.close();


/*   Eigen::Matrix4f frame_to_map = Eigen::Matrix4f::Identity();*/

   //frame_to_map(0,0) = frame_2.odometry(0,0);
   //frame_to_map(0,1) = frame_2.odometry(0,1);
   //frame_to_map(0,2) = frame_2.odometry(0,2);
   //frame_to_map(0,3) = frame_2.odometry(0,3);

   //frame_to_map(1,0) = frame_2.odometry(1,0);
   //frame_to_map(1,1) = frame_2.odometry(1,1);
   //frame_to_map(1,2) = frame_2.odometry(1,2);
   //frame_to_map(1,3) = frame_2.odometry(1,3);

   //frame_to_map(2,0) = frame_2.odometry(2,0);
   //frame_to_map(2,1) = frame_2.odometry(2,1);
   //frame_to_map(2,2) = frame_2.odometry(2,2);
   //frame_to_map(2,3) = frame_2.odometry(2,3);

   //PointCloud cloud_save_trans;

   //pcl::transformPointCloud(*cloud_save,cloud_save_trans,frame_to_map);

   pcl::PCDWriter writer;
   frame_1.cloud_transformed->width = frame_1.cloud_transformed->points.size();
   frame_1.cloud_transformed->height = 1;
   cloud_save->width = cloud_save->points.size();
   cloud_save->height = 1;
  //if(!cloud_save.points.empty() && !frame_1.cloud_transformed->points.empty())

   if(!cloud_save->points.empty())
   {
     ss.str("");
     ss << output_folder << "/cloud_trans_robust_a_" << frame_1.frame_id << ".pcd";
     writer.write(ss.str(),*frame_1.cloud_transformed,true);
     ss.str("");
     ss << output_folder << "/cloud_save_robust_a_" << frame_1.frame_id << ".pcd";
     writer.write(ss.str(),*cloud_save,true);
   }
 }

}
