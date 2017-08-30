#ifndef MeshTriangulationCPU_H
#define MeshTriangulationCPU_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <iostream>

using namespace std;


class MeshTriangulationCPU
{
public:
MeshTriangulationCPU();
void cal_mesh(const PointCloud::Ptr &cloud_input ,pcl::PolygonMesh &triangles_cpu , const float search_radius , const int mesh_max_neighbors ,const float mu);

//void cal_mesh(const PointCloud &cloud_input , const NormalCloud & cloud_normal, pcl::PolygonMesh &triangles_cpu , const float search_radius , const int mesh_max_neighbors ,const float mu);




};


#include "mesh_triangulation_cpu.hpp"
#endif

