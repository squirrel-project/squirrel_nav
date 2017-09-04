// Copyright (c) 2017, Ayush Dewan and Wolfram Burgard
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


#include "ros/ros.h"
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeNode.h>
#include <iostream>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace octomap;
using namespace std;
using namespace message_filters;
typedef sync_policies::ApproximateTime<octomap_msgs::Octomap,octomap_msgs::Octomap> MySyncPolicy;
class PublishOctomap
{

  private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher publisher;
    pcl::KdTreeFLANN <pcl::PointXYZ> kdtree;
    pcl::PCDReader reader;
    message_filters::Subscriber<octomap_msgs::Octomap> binary_sub;
    message_filters::Subscriber<octomap_msgs::Octomap> full_sub;
    Synchronizer<MySyncPolicy> sync;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    ColorOcTree *original_tree;
  public:

    string map_filename;
    void load_map()
    {
      original_tree = new ColorOcTree(0.05);
      original_tree->readBinary(map_filename);
      for(ColorOcTree::leaf_iterator it = original_tree->begin_leafs(),end = original_tree->end_leafs(); it!= end; ++it)
      {
        if(it->getOccupancy() > 0.5)
        {
          point3d point = it.getCoordinate();
          pcl::PointXYZ pt_pcl;
          pt_pcl.x = point.x();
          pt_pcl.y = point.y();
          pt_pcl.z = point.z();
          cloud->points.push_back(pt_pcl);
        }

        it->setColor(0,255,0);
      }

    //  fprintf(stderr,"%d\n",cloud->points.size());
      kdtree.setInputCloud(cloud);


    }
    PublishOctomap():cloud(new pcl::PointCloud<pcl::PointXYZ>),binary_sub(nh,"octomap_binary",50),full_sub(nh, "octomap_full", 50),sync(MySyncPolicy(10), binary_sub, full_sub)
    {

      publisher = nh.advertise<octomap_msgs::Octomap>("/octomap_full_color",10);//Publising the filtered pointcloud
//      reader.read("/home/dewan/octo_maps/18012017.pcd",*cloud);
      sync.registerCallback(boost::bind(&PublishOctomap::callback,this, _1, _2));

    }
    void callback(const octomap_msgs::OctomapConstPtr &map_binary, const octomap_msgs::OctomapConstPtr &map_full)
    {

      octomap::OcTree* octree = new octomap::OcTree(map_full->resolution);

      delete original_tree;
      original_tree = new ColorOcTree(0.05);
      //octomap::ColorOcTree* octree_color = new octomap::ColorOcTree(map_binary->resolution);
      std::stringstream datastream;
      if(map_binary->data.size() > 0)
      {
        datastream.write((const char*) &map_binary->data[0], map_binary->data.size());
        original_tree->readBinaryData(datastream);
        //octree_color->readBinaryData(datastream);
      }
      if(map_full->data.size() > 0)
      {
        octomap::AbstractOcTree* tree = octomap::AbstractOcTree::createTree(map_full->id,map_full->resolution);
        datastream.write((const char*) &map_full->data[0], map_full->data.size());
        tree->readData(datastream);
        octree = dynamic_cast<octomap::OcTree*>(tree);
      }

      for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),end = octree->end_leafs(); it!= end; ++it)
      {
//        fprintf(stderr,"%f\n",it->getOccupancy());
        if(it->getOccupancy() < 0.9)
        {
          octomap::point3d pt = it.getCoordinate();
          original_tree->deleteNode(pt,it.getDepth());
          //octree_color->deleteNode(pt,it.getDepth());
        }
      }
      for(octomap::ColorOcTree::leaf_iterator it = original_tree->begin_leafs(),end = original_tree->end_leafs(); it!= end; ++it)
      {

        octomap::point3d point = it.getCoordinate();
        pcl::PointXYZ pt_pcl;
        pt_pcl.x = point.x();
        pt_pcl.y = point.y();
        pt_pcl.z = point.z();
        std::vector <int> pointIdxRadiusSearch;
        std::vector <float> pointRadiusSquaredDistance;
        if(kdtree.nearestKSearch(pt_pcl,1,pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0)
          if(sqrt(pointRadiusSquaredDistance[0]) < 0.2)
            it->setColor(0,255,0);
          else
            it->setColor(0,0,255);

      //  fprintf(stderr,"%f\n",it->getOccupancy());


      }
           //delete octree;
      delete octree;
    }


    void run()
    {

      while(ros::ok())
      {
        octomap_msgs::Octomap msg_pub;
        octomap_msgs::fullMapToMsg(*original_tree, msg_pub);
        msg_pub.header.frame_id = "map";
        publisher.publish(msg_pub);

        ros::spinOnce();
      }


    }


};




int main(int argc, char **argv)
{
	ros::init(argc, argv, "squirrel_dynamic_filter_publish_color_octomap");

  PublishOctomap publish_octomap;

  publish_octomap.map_filename = argv[1];
  publish_octomap.load_map();
  publish_octomap.run();


	  return 1;

}
