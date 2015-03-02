// RobotLaser.cpp --- 
// 
// Filename: RobotLaser.cpp
// Description: Handling Laser data...
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:06:46 2015 (+0100)
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


#include "squirrel_localizer/RobotLaser.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

using namespace std;

RobotLaser::RobotLaser()
{
  // Empty constructor
}

RobotLaser::RobotLaser(const RobotLaser& l) :
    RawLaser(l)
{
  m_parameters.type = l.m_parameters.type;
  m_parameters.startAngle = l.m_parameters.startAngle;
  m_parameters.fov = l.m_parameters.fov;
  m_parameters.angularResolution = l.m_parameters.angularResolution;
  m_parameters.maxRange = l.m_parameters.maxRange;
  m_parameters.accuracy = l.m_parameters.accuracy;
  m_parameters.remissionMode = l.m_parameters.remissionMode;

  m_ranges.clear();
  for(unsigned int i = 0; i < l.m_ranges.size(); i++) {
    m_ranges.push_back(l.m_ranges.at(i));
  }

  m_remissions.clear();
  for(unsigned int i = 0; i < l.m_remissions.size(); i++) {
    m_remissions.push_back(l.m_remissions.at(i));
  }

  m_laserPose.x = l.m_laserPose.x;
  m_laserPose.y = l.m_laserPose.y;
  m_laserPose.theta = l.m_laserPose.theta;

  m_robotPose.x = l.m_robotPose.x;
  m_robotPose.y = l.m_robotPose.y;
  m_robotPose.theta = l.m_robotPose.theta;

  m_laserTv = l.m_laserTv; 
  m_laserRv = l.m_laserRv; 
  m_forwardSafetyDist = l.m_forwardSafetyDist; 
  m_sideSaftyDist = l.m_sideSaftyDist;
  m_turnAxis = l.m_turnAxis;

  m_timeStamp = l.m_timeStamp;
}

RobotLaser& RobotLaser::RobotLaser::operator=(const RobotLaser& l) {
  if (this != &l) {
    m_parameters.type = l.m_parameters.type;
    m_parameters.startAngle = l.m_parameters.startAngle;
    m_parameters.fov = l.m_parameters.fov;
    m_parameters.angularResolution = l.m_parameters.angularResolution;
    m_parameters.maxRange = l.m_parameters.maxRange;
    m_parameters.accuracy = l.m_parameters.accuracy;
    m_parameters.remissionMode = l.m_parameters.remissionMode;
    
    m_ranges.clear();
    for(unsigned int i = 0; i < l.m_ranges.size(); i++) {
      m_ranges.push_back(l.m_ranges.at(i));
    }
    
    m_remissions.clear();
    for(unsigned int i = 0; i < l.m_remissions.size(); i++) {
      m_remissions.push_back(l.m_remissions.at(i));
    }
    
    m_laserPose.x = l.m_laserPose.x;
    m_laserPose.y = l.m_laserPose.y;
    m_laserPose.theta = l.m_laserPose.theta;
    
    m_robotPose.x = l.m_robotPose.x;
    m_robotPose.y = l.m_robotPose.y;
    m_robotPose.theta = l.m_robotPose.theta;
    
    m_laserTv = l.m_laserTv; 
    m_laserRv = l.m_laserRv; 
    m_forwardSafetyDist = l.m_forwardSafetyDist; 
    m_sideSaftyDist = l.m_sideSaftyDist;
    m_turnAxis = l.m_turnAxis;
    
    m_timeStamp = l.m_timeStamp;
  }
  
  return *this;
}

RobotLaser::~RobotLaser() {
  m_ranges.clear();
  m_remissions.clear();
}

void RobotLaser::setValues(std::string logEntry) {
  istringstream inStream;
  inStream.str(logEntry);
  
  char messageName[64];
  inStream >> messageName;
    
  inStream >> m_parameters.type;
  inStream >> m_parameters.startAngle;
  inStream >> m_parameters.fov;
  inStream >> m_parameters.angularResolution;
  inStream >> m_parameters.maxRange;
  inStream >> m_parameters.accuracy;
  inStream >> m_parameters.remissionMode;
  
  int numReadings;
  inStream >> numReadings;

  float distance;
  m_ranges.clear();
  for(int i = 0; i < numReadings; i++) {
    inStream >> distance;
    m_ranges.push_back(distance);
  }
  
  int numRemissions;
  inStream >> numRemissions;

  float remission;
  m_remissions.clear();
  for(int i = 0; i < numRemissions; i++) {
    inStream >> remission;
    m_remissions.push_back(remission);
  }

  inStream >> m_laserPose.x;
  inStream >> m_laserPose.y;
  inStream >> m_laserPose.theta;

  inStream >> m_robotPose.x;
  inStream >> m_robotPose.y;
  inStream >> m_robotPose.theta;

  inStream >> m_laserTv; 
  inStream >> m_laserRv; 
  inStream >> m_forwardSafetyDist; 
  inStream >> m_sideSaftyDist;
  inStream >> m_turnAxis;
  
  inStream >> m_timeStamp; // ipc timestamp

  /*
    inStream >> messageName; // hostname

    float dummyFloat;
    inStream >> dummyFloat; // logger timestamp
  */
}

std::string RobotLaser::toString() {
  ostringstream s;
  
  s << "ROBOTLASER1";
  s << " " << m_parameters.type;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_parameters.startAngle;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_parameters.fov;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_parameters.angularResolution;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_parameters.maxRange;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_parameters.accuracy;
  s << " " << m_parameters.remissionMode;

  s << " " << m_ranges.size();  
  for(unsigned int i = 0; i < m_ranges.size(); i++) {
    s << " " << setiosflags(ios::fixed) << setprecision(3) << m_ranges[i];
  }

  s << " " << m_remissions.size();
  for(unsigned int i = 0; i < m_remissions.size(); i++) {
    s << " " << setiosflags(ios::fixed) << setprecision(3) << m_remissions[i];
  }

  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_laserPose.x;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_laserPose.y;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_laserPose.theta;

  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_robotPose.x;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_robotPose.y;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_robotPose.theta;

  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_laserTv; 
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_laserRv; 
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_forwardSafetyDist; 
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_sideSaftyDist;
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_turnAxis;

  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_timeStamp; // ipc_timestamp
  s << " hostname";
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_timeStamp; // logger_timestamp
  s << endl;

  return s.str();
}

std::vector<Point> RobotLaser::getRemissionPoints() const {
  std::vector<Point> points;
  
  double angle = m_parameters.startAngle + m_laserPose.theta;
  
  for (unsigned int i = 0; i < m_remissions.size(); i++){
    if (m_ranges[i] < m_parameters.maxRange && m_remissions[i] > 0) {
      double x = m_ranges[i]*cos(angle) + m_laserPose.x;
      double y = m_ranges[i]*sin(angle) + m_laserPose.y;
      
      points.push_back( Point(x, y) );
    }
    
    angle += m_parameters.angularResolution;
  }
  
  return points;
}

vector<Point> RobotLaser::getScanPoints() const {
  std::vector<Point> points;
  
  double angle = m_parameters.startAngle;

  for (unsigned int i = 0; i < m_ranges.size(); i++){
    if (m_ranges[i] < m_parameters.maxRange ) {

      double x = m_ranges[i]*cos(angle);
      double y = m_ranges[i]*sin(angle);
 
      points.push_back( Point(x, y) );
    }
    
    angle += m_parameters.angularResolution;
  }
  
  return points;
}

vector<Point> RobotLaser::getMeasuredPoints() const {
  std::vector<Point> points;
  
  double angle = m_parameters.startAngle + m_laserPose.theta;

  for (unsigned int i = 0; i < m_ranges.size(); i++){
    if (m_ranges[i] < m_parameters.maxRange ) {

      double x = m_ranges[i]*cos(angle) + m_laserPose.x;
      double y = m_ranges[i]*sin(angle) + m_laserPose.y;
 
      points.push_back( Point(x, y) );
    }

    angle += m_parameters.angularResolution;
  }

  return points;
}

// 
// RobotLaser.cpp ends here
