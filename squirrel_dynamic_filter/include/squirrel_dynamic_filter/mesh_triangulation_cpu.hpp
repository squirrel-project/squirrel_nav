#ifndef MeshTriangulationCPU_HPP
#define MeshTriangulationCPU_HPP

#include "mesh_triangulation_cpu.h"

inline MeshTriangulationCPU::MeshTriangulationCPU(){};

inline void MeshTriangulationCPU::cal_mesh(const PointCloud::Ptr &cloud_input ,pcl::PolygonMesh &triangles_cpu , const float search_radius , const int mesh_max_neighbors ,const float mu)
{ 

  cout << "input_mesh" << cloud_input->points.size() << endl;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_input);
  n.setInputCloud (cloud_input);
  n.setSearchMethod (tree);
  n.setKSearch (30);
//  n.setRadiusSearch (0.1);
  n.compute (*normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_input, *normals, *cloud_with_normals);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  gp3.setSearchRadius (search_radius);
  gp3.setMu (mu);
  gp3.setMaximumNearestNeighbors (mesh_max_neighbors);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles_cpu);
  std::vector<int> parts = gp3.getPartIDs();

  int counter = 0;
  for(size_t i = 0; i < parts.size(); ++i)
  {


   if(parts[i] == -1)
    counter+=1;



  } 


  cout << triangles_cpu.polygons.size() << "," << cloud_input->points.size() << endl;
}
#endif























/*
void cal_mesh(const PointCloud &cloud_input , const NormalCloud & cloud_normal, pcl::PolygonMesh &triangles_cpu , const float search_radius , const int mesh_max_neighbors ,const float mu)
{










}

*/


