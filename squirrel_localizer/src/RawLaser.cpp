// RawLaser.cpp --- 
// 
// Filename: RawLaser.cpp
// Description: Handling laser data...
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:04:52 2015 (+0100)
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

#include "squirrel_localizer/RawLaser.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

using namespace std;

RawLaser::RawLaser() {
}

RawLaser::RawLaser(const RawLaser& l) {
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

  m_timeStamp = l.m_timeStamp;
}

RawLaser& RawLaser::RawLaser::operator=(const RawLaser& l) {
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
    
    m_timeStamp = l.m_timeStamp;
  }

  return *this;
}

RawLaser::~RawLaser() {
  m_ranges.clear();
  m_remissions.clear();
}

void RawLaser::setValues(std::string logEntry) {
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
  
  inStream >> m_timeStamp; // ipc_timestamp
}

std::string RawLaser::toString() {
  ostringstream s;
  
  s << "RAWLASER1";
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

  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_timeStamp; // ipc_timestamp
  s << " hostname";
  s << " " << setiosflags(ios::fixed) << setprecision(6) << m_timeStamp; // logger_timestamp
  s << endl;

  return s.str();
}

vector<Point> RawLaser::getRemissionPoints() const {
  std::vector<Point> points;
  
  double angle = m_parameters.startAngle;

  for (unsigned int i = 0; i < m_remissions.size(); i++){
    if (m_ranges[i] < m_parameters.maxRange && m_remissions[i] > 0) {
      double x = m_ranges[i]*cos(angle);
      double y = m_ranges[i]*sin(angle);
 
      points.push_back( Point(x, y) );
    }

    angle += m_parameters.angularResolution;
  }

  return points;
}

vector<Point> RawLaser::getScanPoints() const {
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

// 
// RawLaser.cpp ends here
