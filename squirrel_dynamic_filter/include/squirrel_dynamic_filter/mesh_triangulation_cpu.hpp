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


