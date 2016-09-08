#include "dynamic_filter_node.h"
using namespace Eigen;
using namespace std;

#if 0
void DynamicFilter::EstimateMotion2(const std::vector <int> &index_query,const std::vector <int> &index_match)
{
 PointCloud cloud_t_current;
 Isometry3D Identity;
 Identity.setIdentity();
 std::vector <Isometry3D> motion_init_current(frame_1.raw_input->points.size(),Identity);
////
 
 frame_1.neighbours.clear();
 frame_1.neighbours.resize(frame_1.raw_input->points.size());


 MeshTriangulationCPU estimate_mesh;
 pcl::PolygonMesh triangles;
 estimate_mesh.cal_mesh(frame_1.raw_input,triangles,0.2,200,10.5);

 for(size_t i = 0 ; i<triangles.polygons.size() ; ++i)
 {
  frame_1.neighbours[triangles.polygons[i].vertices[0]].push_back(triangles.polygons[i].vertices[1]);
  frame_1.neighbours[triangles.polygons[i].vertices[0]].push_back(triangles.polygons[i].vertices[2]);
	   
  frame_1.neighbours[triangles.polygons[i].vertices[1]].push_back(triangles.polygons[i].vertices[0]);
  frame_1.neighbours[triangles.polygons[i].vertices[1]].push_back(triangles.polygons[i].vertices[2]);
  
  frame_1.neighbours[triangles.polygons[i].vertices[2]].push_back(triangles.polygons[i].vertices[0]);
  frame_1.neighbours[triangles.polygons[i].vertices[2]].push_back(triangles.polygons[i].vertices[1]);
 
 }
 

 ////if they are less than 3 keypoints, do not opitmize
// if(index_query.size()<3)
 // continue;

 PointCloud::Ptr kp (new PointCloud);
///From original cloud copy the cluster and keypoints in the cluster  

 pcl::copyPointCloud(*frame_1.raw_input,index_query,*kp);

 BlockSolverX::LinearSolverType * linearSolver(new LinearSolverCSparse<BlockSolverX::PoseMatrixType>());
 BlockSolverX* blockSolver(new BlockSolverX(linearSolver));
 OptimizationAlgorithmLevenberg* optimizationAlgorithm(new OptimizationAlgorithmLevenberg(blockSolver));
 SparseOptimizer optimizer;
 optimizer.setVerbose(false);
 optimizer.setAlgorithm(optimizationAlgorithm);
////Initialize the vertextes of the graph. If the first frame each vertex's
//initial value is identity otherwise the vertex's initial value is the
//previoulsy estimated motion

///If first frame
 if(frame_1.motion_init.empty())
  {
   for(size_t k = 0; k < frame_1.raw_input->points.size() ; ++k)
   {
    Vector3D position_input;
    Vector3f pos = frame_1.raw_input->points[k].getVector3fMap();
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
  est.setInputSource (frame_1.raw_input);
  est.setInputTarget (frame_1.cloud_transformed);
  est.determineReciprocalCorrespondences (all_correspondences);
  std::vector <bool> is_correspondence ( frame_1.raw_input->points.size() , false);
  std::vector <int> match ( frame_1.raw_input->points.size() , -1);
  for ( size_t j = 0; j < all_correspondences.size(); ++j)
  {
   is_correspondence[all_correspondences[j].index_query]=true;
   match[all_correspondences[j].index_query] = all_correspondences[j].index_match;
  }
  for(size_t k = 0; k < frame_1.raw_input->points.size() ; ++k)
  {
   Vector3D position_input;
   Vector3f pos = frame_1.raw_input->points[k].getVector3fMap();
   position_input[0] = pos[0];
   position_input[1] = pos[1];
   position_input[2] = pos[2];
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
 ///Initializing unary edges connecting correspondences

 for(size_t k = 0;  k < index_match.size(); ++k)
  {
   Vector3D position_input;
   Vector3f pos = frame_2.raw_input->points[index_match[k]].getVector3fMap();
   Vector3f pos_1 = frame_1.raw_input->points[index_query[k]].getVector3fMap();
   position_input[0] = pos[0];
   position_input[1] = pos[1];
   position_input[2] = pos[2];
   EdgeICP* correspondence=new EdgeICP;
   correspondence->vertices()[0] = optimizer.vertex(index_query[k]);
   correspondence->setMeasurement(position_input);
   MatrixXd information=MatrixXd::Identity(3,3);
   correspondence->setInformation(information);
   optimizer.addEdge(correspondence);
  }
 MatrixXint edge_matrix = MatrixXint::Zero(frame_1.raw_input->points.size(),frame_1.raw_input->points.size());
 pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
 kdtree.setInputCloud (kp);
 for(size_t k = 0 ; k < kp->points.size() ; ++k)
 {
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  if ( kdtree.radiusSearch (kp->points[k],keypoint_radius,pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0 )
  {
   int n = 10;
   if(n > pointIdxRadiusSearch.size())
    n =  pointIdxRadiusSearch.size();
   for(size_t j = 1;j < n;++j)
   {
    if(k == pointIdxRadiusSearch[j])
     continue;
    
    
    if(edge_matrix(index_query[k],index_query[pointIdxRadiusSearch[j]])==0)
    {

     MatrixXd information=MatrixXd::Identity(6,6);
     Edge* odometry = new Edge;
     odometry->vertices()[0] = optimizer.vertex(index_query[k]);
     odometry->vertices()[1] = optimizer.vertex(index_query[pointIdxRadiusSearch[j]]);
     odometry->setMeasurement(Isometry3D::Identity());
     
     odometry->setInformation(information);

     optimizer.addEdge(odometry);
     edge_matrix(index_query[k],index_query[pointIdxRadiusSearch[j]])=edge_matrix(index_query[pointIdxRadiusSearch[j]],index_query[k])=1;

   }
   else
     continue;
   }
  }
 }
// Connecting neighboring points for binaey edges
  for ( size_t k = 0; k < frame_1.raw_input->points.size() ; ++k)
  {
   int check_neighbours = 0;
   for( size_t j = 0; j < frame_1.neighbours[k].size() ; ++j)
   {
  
    if(edge_matrix(k,frame_1.neighbours[k][j]) == 0)
    {
     MatrixXd information=MatrixXd::Identity(6,6);
     Edge* odometry = new Edge;
     odometry->vertices()[0] = optimizer.vertex(k);
     odometry->vertices()[1] = optimizer.vertex(frame_1.neighbours[k][j]);
     odometry->setMeasurement(Isometry3D::Identity());
     odometry->setInformation(information);
     optimizer.addEdge(odometry);
     edge_matrix(k,frame_1.neighbours[k][j]) = edge_matrix(frame_1.neighbours[k][j],k)=1;
    }
   }
   
  }

  optimizer.initializeOptimization();
  SparseOptimizerTerminateAction* terminateAction = 0;
  terminateAction = new SparseOptimizerTerminateAction;
  terminateAction->setGainThreshold(0.08);
  terminateAction->setMaxIterations(5);
  optimizer.addPostIterationAction(terminateAction);
  optimizer.optimize(10);
////Second round of optimzation
  g2o::HyperGraph::VertexIDMap ver = optimizer.vertices();
  g2o::HyperGraph::EdgeSet edge_set = optimizer.edges();
/*
  for (HyperGraph::EdgeSet::const_iterator eit = edge_set.begin(); eit != edge_set.end(); ++eit) 
   {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*eit);
    if(e->dimension()==3)
    {
      g2o::RobustKernelSaturated* rk=new g2o::RobustKernelSaturated;
      rk->setDelta(std::sqrt(0.05));
      e->setRobustKernel(rk);
    }
   }
   optimizer.initializeOptimization();
   optimizer.addPostIterationAction(terminateAction);
   optimizer.optimize(10);
*/

   ver = optimizer.vertices();
   for(std::unordered_map<int,HyperGraph::Vertex*>::iterator it=ver.begin();it!=ver.end();++it)
   {
    g2o::OptimizableGraph::Vertex* v=static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
    std::vector <double> data; 
    v->getMinimalEstimateData(data);
    Vector6d data_new;
    
    data_new(0) = data[0];data_new(1) = data[1];data_new(2) = data[2];data_new(3) = data[3];data_new(4) = data[4];data_new(5) = data[5];
  //  cout << data_new.lpNorm<2>() << endl;;
    Isometry3D motion = g2o::internal::fromVectorMQT(data_new);
//    cout << cluster[v->id()] << "," << motion_init_current.size() << endl;
    motion_init_current[v->id()] = motion;
   }

    



 frame_1.cloud_transformed->points.clear();
 frame_1.motion_init.clear();
 PointCloud cloud_save;
 ss.str("");
 ss << output_folder << "motion_a" << "_" << frame_1.frame_id << ".csv";
 ofstream write_trans(ss.str().c_str());
 ss.str("");
 ss << output_folder << "intensity" << "_" << frame_1.frame_id << ".csv";
 ofstream write_intensity(ss.str().c_str());
 ss.str("");
 ss << output_folder << "intensity_z" << "_" << frame_1.frame_id << ".csv";
 ofstream write_intensity_z(ss.str().c_str());


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
//   Isometry3D motion_diff = odometry_diff;
  
   Eigen::Vector2d trans(motion_diff(0,3),motion_diff(1,3));

   write_intensity << trans.lpNorm<2>() << endl;

   Eigen::Vector3d trans_z(motion_diff(0,3),motion_diff(1,3),motion_diff(2,3));

   write_intensity_z << trans_z.lpNorm<2>() << endl;


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
   cloud_save.points.push_back(point_cloud);
 
   write_trans << motion(0) << "," << motion(1) << "," <<motion(2) << "," << motion(3) << "," << motion(4) << "," << motion(5) << "," << motion[6] << endl;
  }
 counter +=1;
 }
 cout << "size compare:" << frame_1.raw_input->points.size() << "," << cloud_save.points.size() << endl;

 write_trans.close();
 pcl::PCDWriter writer;
 frame_1.cloud_transformed->width = frame_1.cloud_transformed->points.size();
 frame_1.cloud_transformed->height = 1;
 cloud_save.width = cloud_save.points.size();
 cloud_save.height = 1;
 //if(!cloud_save.points.empty() && !frame_1.cloud_transformed->points.empty())
 if(!cloud_save.points.empty())
 {
  ss.str("");
  ss << output_folder << "/cloud_trans_robust_a_" << frame_1.frame_id << ".pcd";
  writer.write(ss.str(),*frame_1.cloud_transformed,true); 

  ss.str("");
  ss << output_folder << "/cloud_save_robust_a_" << frame_1.frame_id << ".pcd";
  writer.write(ss.str(),cloud_save,true); 
 }
 
 //measure_time end_time = SystemClock::now();
 //TimeDiff t_d = end_time - start_time;
// motion_time = t_d.count();





}














#endif






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
    if(cluster[k]==index_query[j])
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
////Initialize the vertextes of the graph. If the first frame each vertex's
//initial value is identity otherwise the vertex's initial value is the
//previoulsy estimated motion

///If first frame
/*
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
*/
 
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
   //vertex_test->setToOriginImpl();
   vertex_test->setPosition(position_input);
   optimizer.addVertex(vertex_test);
  }

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
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
/*
  kdtree.setInputCloud (kp);
  for(size_t k = 0 ; k < kp->points.size() ; ++k)
  {
   std::vector<int> pointIdxRadiusSearch;
   std::vector<float> pointRadiusSquaredDistance;
   if ( kdtree.radiusSearch (kp->points[k],keypoint_radius,pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0 )
   {
    int n = 10;
    if(n > pointIdxRadiusSearch.size())
     n =  pointIdxRadiusSearch.size();
    for(size_t j = 1;j < n;++j)
    {
     if(k == pointIdxRadiusSearch[j])
      continue;
     
     
     if(edge_matrix(index_query_filter[k],index_query_filter[pointIdxRadiusSearch[j]])==0)
     {

      MatrixXd information=MatrixXd::Identity(6,6);
      Edge* odometry = new Edge;
      odometry->vertices()[0] = optimizer.vertex(index_query_filter[k]);
      odometry->vertices()[1] = optimizer.vertex(index_query_filter[pointIdxRadiusSearch[j]]);
      odometry->setMeasurement(Isometry3D::Identity());
      
      odometry->setInformation(information);

      optimizer.addEdge(odometry);
      edge_matrix(index_query_filter[k],index_query_filter[pointIdxRadiusSearch[j]])=edge_matrix(index_query_filter[pointIdxRadiusSearch[j]],index_query_filter[k])=1;

    }
    else
      continue;
    }
   }
  }
*/
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
/*
   for (HyperGraph::EdgeSet::const_iterator eit = edge_set.begin(); eit != edge_set.end(); ++eit) 
		  {
     OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*eit);
     if(e->dimension()==3)
     {
       g2o::RobustKernelSaturated* rk=new g2o::RobustKernelSaturated;
       rk->setDelta(std::sqrt(0.05));
       e->setRobustKernel(rk);
     }
    }
    optimizer.initializeOptimization();
    optimizer.addPostIterationAction(terminateAction);
    optimizer.optimize(10);
*/

    ver = optimizer.vertices();
    for(std::unordered_map<int,HyperGraph::Vertex*>::iterator it=ver.begin();it!=ver.end();++it)
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
 

 PointCloud::Ptr score(new PointCloud);
 score->points = frame_1.cloud_transformed->points;
 frame_1.cloud_transformed->points.clear();
 frame_1.motion_init.clear();
 PointCloud::Ptr cloud_save(new PointCloud);
 ss.str("");
 ss << output_folder << "motion_a" << "_" << frame_1.frame_id << ".csv";
 ofstream write_trans(ss.str().c_str());
 ss.str("");
 ss << output_folder << "intensity" << "_" << frame_1.frame_id << ".csv";
 ofstream write_intensity(ss.str().c_str());
 ss.str("");
 ss << output_folder << "intensity_z" << "_" << frame_1.frame_id << ".csv";
 ofstream write_intensity_z(ss.str().c_str());


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
//   Isometry3D motion_diff = odometry_diff;
  
   Eigen::Vector2d trans(motion_diff(0,3),motion_diff(1,3));

   write_intensity << trans.lpNorm<2>() << endl;

   Eigen::Vector3d trans_z(motion_diff(0,3),motion_diff(1,3),motion_diff(2,3));

//   write_intensity_z << trans_z.lpNorm<2>() << endl;


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
 
   write_trans << motion(0) << "," << motion(1) << "," <<motion(2) << "," << motion(3) << "," << motion(4) << "," << motion(5) << "," << motion[6] << endl;
  }
 counter +=1;
 }
 if(cloud_save->points.size() > 0 && score->points.size() >0 )
 {
  bool check = false;
  if(frame_1.prior_dynamic.empty())
   check = true;
  DynamicScore(cloud_save,check,score); 

  int count_point = 0; 
  for(auto &score:frame_1.prior_dynamic)
  {
   if(score < 0.15)
    cloud_dynamic.points.push_back(cloud_save->points[count_point]);
   write_intensity_z << score << endl;

   count_point += 1;

  }
 }
 else
 {
  fprintf(stderr,"%d,%d\n",cloud_save->points.size(),score->points.size());
  frame_1.prior_dynamic.clear();
//  frame_1.cloud_transformed->points.clear();

 }
 cout << "size compare:" << frame_1.raw_input->points.size() << "," << cloud_save->points.size() << endl;

 write_trans.close();
 pcl::PCDWriter writer;
 frame_1.cloud_transformed->width = frame_1.cloud_transformed->points.size();
 frame_1.cloud_transformed->height = 1;
 cloud_save->width = cloud_save->points.size();
 cloud_save->height = 1;
 //if(!cloud_save.points.empty() && !frame_1.cloud_transformed->points.empty())
 if(!cloud_save->points.empty())
 {
 // ss.str("");
//  ss << output_folder << "/cloud_trans_robust_a_" << frame_1.frame_id << ".pcd";
 // writer.write(ss.str(),*frame_1.cloud_transformed,true); 

  ss.str("");
  ss << output_folder << "/cloud_save_robust_a_" << frame_1.frame_id << ".pcd";
  writer.write(ss.str(),*cloud_save,true); 
 }
 
 measure_time end_time = SystemClock::now();
 TimeDiff t_d = end_time - start_time;
 motion_time = t_d.count();





}
