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

#ifndef EuclideanClusteringMesh_HPP
#define EuclideanClusteringMesh_HPP

#include "EuclideanClusteringMesh.h"

inline EuclideanClusteringMesh::EuclideanClusteringMesh(){}

inline void EuclideanClusteringMesh::clustering( const pcl::PolygonMesh &triangles , const int min_cluster_size , std::vector< std::vector <int> > &clusters,const PointCloud::Ptr &scan_1)
{
 
 
	std::vector< std::vector <int> > neighbours;
 neighbours.resize(scan_1->points.size());



 for(size_t i = 0 ; i<triangles.polygons.size() ; ++i)
 {
  neighbours[triangles.polygons[i].vertices[0]].push_back(triangles.polygons[i].vertices[1]);
  neighbours[triangles.polygons[i].vertices[0]].push_back(triangles.polygons[i].vertices[2]);
	   
  neighbours[triangles.polygons[i].vertices[1]].push_back(triangles.polygons[i].vertices[0]);
  neighbours[triangles.polygons[i].vertices[1]].push_back(triangles.polygons[i].vertices[2]);
  
  neighbours[triangles.polygons[i].vertices[2]].push_back(triangles.polygons[i].vertices[0]);
  neighbours[triangles.polygons[i].vertices[2]].push_back(triangles.polygons[i].vertices[1]);
 
 }
  
   //cout<<"done with neighours"<<endl;

 std::vector <bool> processed (scan_1->points.size (), false);
 std::vector <int> nn_indices;
 int counter=0;
 for (size_t i = 0; i < scan_1->points.size (); ++i)
 {
  if (processed[i])
   continue;
  std::vector <int> seed_queue;
  int sq_idx = 0;
  seed_queue.push_back (static_cast<int> (i));
  processed[i] = true;
  while (sq_idx < static_cast<int> (seed_queue.size ()))
  {
   if(neighbours[seed_queue[sq_idx]].size()==0)        
   {
    counter++;
    sq_idx++;
    continue;
		 }
   for (size_t j = 0; j < neighbours[seed_queue[sq_idx]].size(); ++j) 
		 {
    if (processed[neighbours[seed_queue[sq_idx]][j]])                 
     continue;
    processed[neighbours[seed_queue[sq_idx]][j]] = true;
    seed_queue.push_back (neighbours[seed_queue[sq_idx]][j]);
		 }
   sq_idx++;
  }
	    
  if(seed_queue.size()>min_cluster_size) 
   clusters.push_back(seed_queue);
  seed_queue.resize(0);
 }

}


inline void EuclideanClusteringMesh::extract_clusters(const PointCloud::Ptr &cloud_input , std::vector< std::vector <int > > &clusters , const float search_radius , const int max_neighbors,const int cloud_size, const int min_size)
{
 MeshTriangulationCPU estimate_mesh;
 pcl::PolygonMesh triangles;
 std::vector <int> indices;
 estimate_mesh.cal_mesh(cloud_input,triangles,search_radius,max_neighbors,10.5);
 clustering(triangles,min_size,clusters,cloud_input);
}
inline void EuclideanClusteringMesh::clustering( const pcl::PolygonMesh &triangles , const int min_cluster_size , std::vector< std::vector <int> > &clusters,std::vector< std::vector <int > > &neighbours,const PointCloud::Ptr &scan_1)
{
 
 
 neighbours.resize(scan_1->points.size());



 for(size_t i = 0 ; i<triangles.polygons.size() ; ++i)
 {
  neighbours[triangles.polygons[i].vertices[0]].push_back(triangles.polygons[i].vertices[1]);
  neighbours[triangles.polygons[i].vertices[0]].push_back(triangles.polygons[i].vertices[2]);
	   
  neighbours[triangles.polygons[i].vertices[1]].push_back(triangles.polygons[i].vertices[0]);
  neighbours[triangles.polygons[i].vertices[1]].push_back(triangles.polygons[i].vertices[2]);
  
  neighbours[triangles.polygons[i].vertices[2]].push_back(triangles.polygons[i].vertices[0]);
  neighbours[triangles.polygons[i].vertices[2]].push_back(triangles.polygons[i].vertices[1]);
 
 }
  
   //cout<<"done with neighours"<<endl;

 std::vector <bool> processed (scan_1->points.size (), false);
 std::vector <int> nn_indices;
 int counter=0;
 for (size_t i = 0; i < scan_1->points.size (); ++i)
 {
  if (processed[i])
   continue;
  std::vector <int> seed_queue;
  int sq_idx = 0;
  seed_queue.push_back (static_cast<int> (i));
  processed[i] = true;
  while (sq_idx < static_cast<int> (seed_queue.size ()))
  {
   if(neighbours[seed_queue[sq_idx]].size()==0)        
   {
    counter++;
    sq_idx++;
    continue;
		 }
   for (size_t j = 0; j < neighbours[seed_queue[sq_idx]].size(); ++j) 
		 {
    if (processed[neighbours[seed_queue[sq_idx]][j]])                 
     continue;
    processed[neighbours[seed_queue[sq_idx]][j]] = true;
    seed_queue.push_back (neighbours[seed_queue[sq_idx]][j]);
		 }
   sq_idx++;
  }
	    
  if(seed_queue.size()>min_cluster_size) 
   clusters.push_back(seed_queue);
  seed_queue.resize(0);
 }

}


inline void EuclideanClusteringMesh::extract_clusters(const PointCloud::Ptr &cloud_input , std::vector< std::vector <int > > &clusters ,std::vector< std::vector <int > > &neighbours, const float search_radius , const int max_neighbors, const int min_size)
{
 MeshTriangulationCPU estimate_mesh;
 pcl::PolygonMesh triangles;
 estimate_mesh.cal_mesh(cloud_input,triangles,search_radius,max_neighbors,10.5);
 clustering(triangles,min_size,clusters,neighbours,cloud_input);


}







#endif
