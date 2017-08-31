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


