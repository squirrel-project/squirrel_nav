#ifndef EuclideanClusteringMesh_H
#define EuclideanClusteringMesh_H
#include "mesh_triangulation_cpu.h"
using namespace std;


class EuclideanClusteringMesh
{
public:
EuclideanClusteringMesh();

void clustering( const pcl::PolygonMesh &triangles , const int min_cluster_size , std::vector< std::vector <int> > &clusters,const PointCloud::Ptr &scan_1);

void extract_clusters(const PointCloud::Ptr &cloud_input , std::vector< std::vector <int > > &clusters , const float search_radius , const int max_neighbors,const int cloud_size,const int min_size);
void clustering( const pcl::PolygonMesh &triangles , const int min_cluster_size , std::vector< std::vector <int> > &clusters,std::vector< std::vector <int > > &neighbours,const PointCloud::Ptr &scan_1);

void extract_clusters(const PointCloud::Ptr &cloud_input , std::vector< std::vector <int > > &clusters ,std::vector< std::vector <int > > &neighbours, const float search_radius , const int max_neighbors,const int min_size);


};


#include"EuclideanClusteringMesh.hpp"
#endif

