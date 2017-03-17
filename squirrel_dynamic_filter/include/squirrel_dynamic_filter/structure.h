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

#include "datatypes_squirrel.h"
#include "edge_unary.h"
#include "edge.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"

struct Frame
{

 PointCloud::Ptr raw_input;
 PointCloud::Ptr cloud_input;///After preprocessing
 PointCloud::Ptr ground;
 SHOTCloud::Ptr feature;
 Isometry3D odometry;
 int frame_id;
 std::vector <Isometry3D> motion_init;
 PointCloud::Ptr cloud_transformed;
 Frame():raw_input(new PointCloud),cloud_input(new PointCloud),ground(new PointCloud),feature(new SHOTCloud),cloud_transformed(new PointCloud)
 {
  frame_id = -1;
 }

 std::vector < std::vector<int> > clusters;
 std::vector < std::vector<int> > neighbours;
 std::vector <int> finite_points;
 std::vector <int> sampled_points;
 std::vector <float> prior_dynamic;
 void clear()
 {  
  raw_input->points.clear();
  cloud_input->points.clear();
  feature->points.clear(); 
  clusters.clear(); 
  finite_points.clear();
  sampled_points.clear();
  neighbours.clear();
  ground->points.clear();
 }
 void copy(Frame &frame)
 {   
  frame.raw_input->points.clear();
  frame.cloud_input->points.clear();
  frame.feature->points.clear();
  frame.finite_points.clear();
  frame.sampled_points.clear();
  frame.clusters.clear();
  frame.neighbours.clear();
  frame.ground->points.clear();
  
  frame.raw_input->points = raw_input->points;
  frame.cloud_input->points = cloud_input->points;
  frame.feature->points = feature->points;
  frame.finite_points = finite_points;
  frame.clusters = clusters;
  frame.frame_id = frame_id;
  frame.neighbours = neighbours;
  frame.ground = ground;
  frame.odometry(0,0) = odometry(0,0);
  frame.odometry(1,0) = odometry(1,0);
  frame.odometry(2,0) = odometry(2,0);
  frame.odometry(3,0) = odometry(3,0);
  frame.odometry(0,1) = odometry(0,1);
  frame.odometry(1,1) = odometry(1,1);
  frame.odometry(2,1) = odometry(2,1);
  frame.odometry(3,1) = odometry(3,1);
  frame.odometry(0,2) = odometry(0,2);
  frame.odometry(1,2) = odometry(1,2);
  frame.odometry(2,2) = odometry(2,2);
  frame.odometry(3,2) = odometry(3,2);
  frame.odometry(0,3) = odometry(0,3);
  frame.odometry(1,3) = odometry(1,3);
  frame.odometry(2,3) = odometry(2,3);
  frame.odometry(3,3) = odometry(3,3);


  

 }

 



 ///Odometry,motion,clsuter
 
};
struct correspondences
{
 int query;
 int match;
 std::vector<int>possible_matches;
 std::vector<float>score;
 std::vector<float>likelihood;
 std::vector<float>probability;
};

