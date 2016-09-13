// ScanMatcher.cpp --- 
// 
// Filename: ScanMatcher.cpp
// Description: Scan mathcing for laser model in particle filter
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de 
// Created: Tue Feb 17 13:43:37 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro 
//   ROS Indigo
// 

// Code:


#include "squirrel_localizer/ScanMatcher.h"

#include <iostream>
#include <cmath>

#include <values.h>

using namespace std;

ScanMatcher::ScanMatcher() {}

ScanMatcher::ScanMatcher(AISNavigation::FloatMap* d_map){
  m_distanceMap = d_map;
  init();
}

ScanMatcher::~ScanMatcher() {

}

void ScanMatcher::init() {
  m_correlativeTranslationalSearchRadius = 0.4;
  m_correlativeTranslationalRadius = (int) (m_correlativeTranslationalSearchRadius /m_distanceMap->resolution());

  m_correlativeRotationalSearchRadius = M_PI/180 * 5; // 0.2;
  m_correlativeRotationalSearchStep = M_PI/720;
  //m_correlativeRotationalSearchStep = M_PI/360;

  m_sparsificationDistance = m_distanceMap->resolution() * m_distanceMap->resolution(); // squared distance

  for (int x = 0; x < m_distanceMap->size().x(); x++){
    for (int y = 0; y < m_distanceMap->size().y(); y++){
      Vector2i p1(x,y);
      if (m_distWorstValue < m_distanceMap->cell(p1)){
        m_distWorstValue = m_distanceMap->cell(p1);
      }
    }
  }


  m_size_i = m_distanceMap->size().x();
  m_size_j = m_distanceMap->size().y();
  m_minPointsNum = 80.0;
}




void ScanMatcher::correlativeScanMatch(const std::vector<Vector3f>& endPoints,
                                       const Pose& estimatedPose, Pose& correctedPose, double& score){

  //Pose estimatedPose = scan.m_laserPose; //TODO remove
  Pose bestPose = estimatedPose; //TODO remove scan.m_laserPose;
  int ci = -1, cj = -1;

  vector<Point> pts;
  for (std::vector<Vector3f>::const_iterator it = endPoints.begin() ; it !=endPoints.end(); it++ ){
    pts.push_back( Point(it->x(), it->y()) );
  }
  vector<Point> sparsifiedPoints = sparsify(pts, m_sparsificationDistance);
  vector<IntPair> cells(sparsifiedPoints.size());

  if (sparsifiedPoints.size() == 0) {
    correctedPose = estimatedPose;
    score = 0.0;

    return;
  }

  double bestScore = cells.size()*m_distWorstValue; // all measurements end up in nirvana
  double worstScore = bestScore;

  Point tp;

  for(double tt = estimatedPose.theta - m_correlativeRotationalSearchRadius;
      tt <= estimatedPose.theta + m_correlativeRotationalSearchRadius;
      tt += m_correlativeRotationalSearchStep) {
    // rotate the points
    for(unsigned int n = 0; n < sparsifiedPoints.size(); n++) {
      double c = cos(tt);
      double s = sin(tt);

      tp.x = sparsifiedPoints[n].x*c - sparsifiedPoints[n].y*s + estimatedPose.x;
      tp.y = sparsifiedPoints[n].x*s + sparsifiedPoints[n].y*c + estimatedPose.y;

      // m_gridMap.global2grid(tp.x, tp.y, &cells[n].i, &cells[n].j);
      Vector2 wp(tp.x,tp.y);
      Vector2i mp = m_distanceMap->world2map(wp);
      cells[n].i = mp.x();
      cells[n].j = mp.y();
    }

    for (int tx = -m_correlativeTranslationalRadius; tx <= m_correlativeTranslationalRadius; tx++){
      for (int ty = -m_correlativeTranslationalRadius; ty <= m_correlativeTranslationalRadius; ty++){

        double s = 0.0;

        // translate the points
        for(unsigned int n = 0; n < cells.size(); n++) {
          ci = cells[n].i + tx;
          cj = cells[n].j + ty;
          Vector2 mp(ci,cj);

          if (isInside(ci, cj)) {
            int cc = cj * m_size_i + ci;
            s += m_distanceMap->cell(mp);
          }
        }

        if (s < bestScore) {
          bestScore = s;
          bestPose.x = estimatedPose.x + (tx) * m_distanceMap->resolution();
          bestPose.y = estimatedPose.y + (ty) * m_distanceMap->resolution();
          bestPose.theta = normalizeAngle(tt);
        }
      }
    }
  }

  correctedPose = bestPose;

  double minPts = m_minPointsNum;
  if (sparsifiedPoints.size() > minPts)
    minPts = sparsifiedPoints.size();

  score = ((worstScore - bestScore) / worstScore)*(sparsifiedPoints.size() / minPts);
}

bool ScanMatcher::isInside(int i, int j) {
  return
      i >= 0
      && j >=0
      && i < m_size_i
             && j < m_size_j;
}


//TODO remove
/*



  ScanMatcher::ScanMatcher(double resolution, int sizeX, int sizeY, int maxHistorySize) {
  init(resolution, sizeX, sizeY, maxHistorySize);
  }

  ScanMatcher::ScanMatcher(const uint32_t& size_x_,const uint32_t& size_y_,
  const double& offset_x_, const double&  offset_y_,
  const float& resolution_, const std::vector<signed char>& data, int maxHistorySize){

  init(size_x_, size_y_, offset_x_, offset_y_, resolution_, data, maxHistorySize);
  }

  void ScanMatcher::init(const uint32_t& size_x_,const uint32_t& size_y_,
  const double& offset_x_, const double&  offset_y_,
  const float& resolution_, const std::vector<signed char>& data, int maxHistorySize) {

  m_maxHistorySize = maxHistorySize;

  m_gridMap.fromInt8Vector(size_x_, size_y_, offset_x_,  offset_y_,resolution_, data );

  m_correlativeTranslationalSearchRadius = 0.2;
  m_correlativeTranslationalRadius = (int) (m_correlativeTranslationalSearchRadius / m_gridMap.m_resolution);

  m_correlativeRotationalSearchRadius = 0.2;
  m_correlativeRotationalSearchStep = M_PI/720;
  //m_correlativeRotationalSearchStep = M_PI/360;

  m_hillClimbingRotationalSearchStep = 0.025;
  m_hillClimbingTranlationalSearchStep = 0.015;

  m_sparsificationDistance = m_gridMap.m_resolution*m_gridMap.m_resolution; // squared distance

  m_minPointsNum = 80.0;
  }

  void ScanMatcher::init(double resolution, int sizeX, int sizeY, int maxHistorySize) {
  m_maxHistorySize = maxHistorySize;

  m_gridMap.init(resolution, sizeX, sizeY);

  m_correlativeTranslationalSearchRadius = 0.2;
  m_correlativeTranslationalRadius = (int) (m_correlativeTranslationalSearchRadius / m_gridMap.m_resolution);
  
  m_correlativeRotationalSearchRadius = 0.2;
  m_correlativeRotationalSearchStep = M_PI/720;
  //m_correlativeRotationalSearchStep = M_PI/360;

  m_hillClimbingRotationalSearchStep = 0.025;
  m_hillClimbingTranlationalSearchStep = 0.015;

  m_sparsificationDistance = m_gridMap.m_resolution*m_gridMap.m_resolution; // squared distance

  m_minPointsNum = 80.0;
  }





  ScanMatcher::~ScanMatcher() {
  m_gridMap.deallocate();
  }

  void ScanMatcher::integrateScan(const RobotLaser& scan, 
  bool centerScan,
  bool limitHistorySize) {
  
  m_scanHistory.push_back(scan);
  
  if (centerScan) { // integrate the last scan in the center of the grid map
  if (limitHistorySize) {
  while ((int) m_scanHistory.size() >= m_maxHistorySize)
  m_scanHistory.pop_front();
  }
    
  bool changeReference = true;
  m_gridMap.clear();
    
  for(int n = m_scanHistory.size() - 1; n >= 0; n--) {
  m_gridMap.integrateScan(m_scanHistory[n], changeReference);
  changeReference = false;
  }
  } else { // integrate the last scan relative to the first scan
    
  if (limitHistorySize && 
  ((int) m_scanHistory.size() >= m_maxHistorySize)) {

  while ((int) m_scanHistory.size() >= m_maxHistorySize)
  m_scanHistory.pop_front();

  bool changeReference = true;
  m_gridMap.clear();
      
  for(unsigned int n = 0; n < m_scanHistory.size(); n++) {
  m_gridMap.integrateScan(m_scanHistory[n], changeReference);
  changeReference = false;
  } 
  } else {
  m_gridMap.integrateScan(scan, m_scanHistory.size() == 1);
  }
  }
  

  m_gridMap.convolve();
  }

  void ScanMatcher::correlativeScanMatch(const RobotLaser& scan, 
  Pose& correctedPose, 
  double& score, 
  double** covariance) {
  Pose estimatedPose = scan.m_laserPose;
  Pose bestPose = scan.m_laserPose;
  int ci = -1, cj = -1;

  vector<Point> pts = scan.getScanPoints();
  vector<Point> sparsifiedPoints = sparsify(pts, m_sparsificationDistance);
  vector<IntPair> cells(sparsifiedPoints.size());

  if (sparsifiedPoints.size() == 0) {
  correctedPose = scan.m_laserPose; 
  score = 0.0;
    
  return;
  }
  
  double bestScore = sparsifiedPoints.size()*m_gridMap.m_convolutionWorstValue; // all measurements end up in nirvana
  double worstScore = bestScore;

  // init structs for covariance calculation
  for(int i = 0; i < 3; i++) {
  c_u[i] = 0.0;
  for(int j = 0; j < 3; j++)
  c_K[i][j] = 0.0;
  }
  c_s = 0.0;

  Point tp;
  
  for(double tt = estimatedPose.theta - m_correlativeRotationalSearchRadius; 
  tt <= estimatedPose.theta + m_correlativeRotationalSearchRadius; 
  tt += m_correlativeRotationalSearchStep) {
    
  // rotate the points
  for(unsigned int n = 0; n < sparsifiedPoints.size(); n++) {
      
  double c = cos(tt);
  double s = sin(tt);
      
  tp.x = sparsifiedPoints[n].x*c - sparsifiedPoints[n].y*s + estimatedPose.x;
  tp.y = sparsifiedPoints[n].x*s + sparsifiedPoints[n].y*c + estimatedPose.y;
      
  m_gridMap.global2grid(tp.x, tp.y, &cells[n].i, &cells[n].j);
  }
    
  for (int tx = -m_correlativeTranslationalRadius; tx <= m_correlativeTranslationalRadius; tx++){
  for (int ty = -m_correlativeTranslationalRadius; ty <= m_correlativeTranslationalRadius; ty++){
	
  double s = 0.0;
	
  // translate the points
  for(unsigned int n = 0; n < cells.size(); n++) {
  ci = cells[n].i + tx;
  cj = cells[n].j + ty;
	  
  if (m_gridMap.isInside(ci, cj)) {
  int cc = cj*m_gridMap.m_size_i + ci;
  s += m_gridMap.m_convolvedCells[cc];
  }
  }
	
  double pose_x = estimatedPose.x + tx*m_gridMap.m_resolution;
  double pose_y = estimatedPose.y + ty*m_gridMap.m_resolution;
  double pose_theta = normalizeAngle(tt);

  if (s < bestScore) {
  bestScore = s;
  bestPose.x = pose_x;
  bestPose.y = pose_y;
  bestPose.theta = pose_theta;
  }

  // covariance stuff
  double l_s = s*m_gridMap.m_resolution*m_gridMap.m_resolution; // convert to meters
  l_s = -0.5*l_s/m_gridMap.m_convolutionStdev2;
  double p_s = exp(l_s);
	
  pose_theta = normalizeAngle(pose_theta - estimatedPose.theta);

  c_K[0][0] += pose_x*pose_x*p_s;
  c_K[0][1] += pose_x*pose_y*p_s;
  c_K[0][2] += pose_x*pose_theta*p_s;
	
  c_K[1][0] += pose_y*pose_x*p_s;
  c_K[1][1] += pose_y*pose_y*p_s;
  c_K[1][2] += pose_y*pose_theta*p_s;
	
  c_K[2][0] += pose_theta*pose_x*p_s;
  c_K[2][1] += pose_theta*pose_y*p_s;
  c_K[2][2] += pose_theta*pose_theta*p_s;

  c_u[0] += pose_x*p_s;
  c_u[1] += pose_y*p_s;
  c_u[2] += pose_theta*p_s;
	
  c_s += p_s;
  }
  }
  }

  for(int i = 0; i < 3; i++)
  for(int j = 0; j < 3; j++)
  c_K[i][j] /= c_s;

  c_uu[0][0] = (c_u[0]*c_u[0])/(c_s*c_s);
  c_uu[0][1] = (c_u[0]*c_u[1])/(c_s*c_s);
  c_uu[0][2] = (c_u[0]*c_u[2])/(c_s*c_s);
  
  c_uu[1][0] = (c_u[1]*c_u[0])/(c_s*c_s);
  c_uu[1][1] = (c_u[1]*c_u[1])/(c_s*c_s);
  c_uu[1][2] = (c_u[1]*c_u[2])/(c_s*c_s);
  
  c_uu[2][0] = (c_u[2]*c_u[0])/(c_s*c_s);
  c_uu[2][1] = (c_u[2]*c_u[1])/(c_s*c_s);
  c_uu[2][2] = (c_u[2]*c_u[2])/(c_s*c_s);

  covariance[0][0] = c_K[0][0] - c_uu[0][0];
  covariance[0][1] = c_K[0][1] - c_uu[0][1];
  covariance[0][2] = c_K[0][2] - c_uu[0][2];

  covariance[1][0] = c_K[1][0] - c_uu[1][0];
  covariance[1][1] = c_K[1][1] - c_uu[1][1];
  covariance[1][2] = c_K[1][2] - c_uu[1][2];

  covariance[2][0] = c_K[2][0] - c_uu[2][0];
  covariance[2][1] = c_K[2][1] - c_uu[2][1];
  covariance[2][2] = c_K[2][2] - c_uu[2][2];

  correctedPose = bestPose; 

  double minPts = m_minPointsNum;
  if (sparsifiedPoints.size() > minPts)
  minPts = sparsifiedPoints.size();

  score = ((worstScore - bestScore) / worstScore)*(sparsifiedPoints.size() / minPts);
  }








  void ScanMatcher::correlativeScanMatch(const RobotLaser& scan, Pose& correctedPose, double& score) {
  Pose estimatedPose = scan.m_laserPose;
  Pose bestPose = scan.m_laserPose;
  int ci = -1, cj = -1;

  vector<Point> pts = scan.getScanPoints();
  vector<Point> sparsifiedPoints = sparsify(pts, m_sparsificationDistance);
  vector<IntPair> cells(sparsifiedPoints.size());

  if (sparsifiedPoints.size() == 0) {
  correctedPose = scan.m_laserPose; 
  score = 0.0;

  return;
  }

  double bestScore = cells.size()*m_gridMap.m_convolutionWorstValue; // all measurements end up in nirvana
  double worstScore = bestScore;

  Point tp;
  
  for(double tt = estimatedPose.theta - m_correlativeRotationalSearchRadius; 
  tt <= estimatedPose.theta + m_correlativeRotationalSearchRadius; 
  tt += m_correlativeRotationalSearchStep) {
    
  // rotate the points
  for(unsigned int n = 0; n < sparsifiedPoints.size(); n++) {
      
  double c = cos(tt);
  double s = sin(tt);
      
  tp.x = sparsifiedPoints[n].x*c - sparsifiedPoints[n].y*s + estimatedPose.x;
  tp.y = sparsifiedPoints[n].x*s + sparsifiedPoints[n].y*c + estimatedPose.y;
      
  m_gridMap.global2grid(tp.x, tp.y, &cells[n].i, &cells[n].j);
  }
    
  for (int tx = -m_correlativeTranslationalRadius; tx <= m_correlativeTranslationalRadius; tx++){
  for (int ty = -m_correlativeTranslationalRadius; ty <= m_correlativeTranslationalRadius; ty++){
	
  double s = 0.0;
	
  // translate the points
  for(unsigned int n = 0; n < cells.size(); n++) {
  ci = cells[n].i + tx;
  cj = cells[n].j + ty;
	  
  if (m_gridMap.isInside(ci, cj)) {
  int cc = cj*m_gridMap.m_size_i + ci;
  s += m_gridMap.m_convolvedCells[cc];
  }
  }
	
  if (s < bestScore) {
  bestScore = s;
  bestPose.x = estimatedPose.x + tx*m_gridMap.m_resolution;
  bestPose.y = estimatedPose.y + ty*m_gridMap.m_resolution;
  bestPose.theta = normalizeAngle(tt);
  }
  }
  }
  }

  correctedPose = bestPose; 

  double minPts = m_minPointsNum;
  if (sparsifiedPoints.size() > minPts)
  minPts = sparsifiedPoints.size();

  score = ((worstScore - bestScore) / worstScore)*(sparsifiedPoints.size() / minPts);
  }
*/


// 
// ScanMatcher.cpp ends here
