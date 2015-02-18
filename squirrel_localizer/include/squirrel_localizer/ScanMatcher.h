// Scanmatcher.h --- 
// 
// Filename: ScanMatcher.h
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:11:01 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//    ROS Hydro
//    ROS Indigo
// 

// Code:

#ifndef SQUIRREL_LOCALIZER_SCANMATCHER_
#define SQUIRREL_LOCALIZER_SCANMATCHER_

#include <vector>
#include <deque>

#include "squirrel_localizer/Geometry.h"
#include "squirrel_localizer/RobotLaser.h"
#include "squirrel_localizer/FloatMap.h"

using namespace AISNavigation;

class ScanMatcher {
 public:
  ScanMatcher();
  ScanMatcher(AISNavigation::FloatMap* d_map);

  //ScanMatcher(double resolution, int sizeX, int sizeY, int maxHistorySize);
  //ScanMatcher(const uint32_t& size_x_,const uint32_t& size_y_,
  //   	  	   const double& offset_x_, const double&  offset_y_,
  //   	  	   const float& resolution_, const std::vector<signed char>& data, int maxHistorySize);
  ~ScanMatcher();

  void init();
  //void init(double resolution, int sizeX, int sizeY, int maxHistorySize);
  //void init(const uint32_t& size_x_,const uint32_t& size_y_,
  //	  	   const double& offset_x_, const double&  offset_y_,
  //	  	   const float& resolution_, const std::vector<signed char>& data, int maxHistorySize);

  //void integrateScan(const RobotLaser& scan, bool centerScan, bool limitHistorySize);

  //void correlativeScanMatch(const RobotLaser& scan, Pose& correctedPose, double& score, double** covariance);
  //void correlativeScanMatch(const RobotLaser& scan, Pose& correctedPose, double& score);
  void correlativeScanMatch(const std::vector<Vector3f>& endPoints,const Pose& estimatedPose, Pose& correctedPose, double& score);
  //void hillClimbingScanMatch(const RobotLaser& scan, Pose& correctedPose, double& score);

  //Pose computeTestPose( int dimension, int step );

  bool isInside(int i, int j);

  // distance map
  AISNavigation::FloatMap *m_distanceMap;

  //scan history
  //int m_maxHistorySize;
  //std::deque<RobotLaser> m_scanHistory;

  // scan matching parameters
  double m_correlativeTranslationalSearchRadius;
  int m_correlativeTranslationalRadius;

  double m_correlativeRotationalSearchRadius;
  double m_correlativeRotationalSearchStep;

  //double m_hillClimbingRotationalSearchStep;
  //double m_hillClimbingTranlationalSearchStep;

  double m_sparsificationDistance;

  double m_minPointsNum;

  double m_distWorstValue; // former m_gridMap.m_convolutionWorstValue

  // map size
  int m_size_i;
  int m_size_j;

  // auxiliary data structures
  double c_K[3][3];
  double c_u[3];
  double c_s;

  double c_uu[3][3];
};

#endif /* SQUIRREL_LOCALIZER_SCANMATCHER_ */

//
// ScanMatcher.h ends here
