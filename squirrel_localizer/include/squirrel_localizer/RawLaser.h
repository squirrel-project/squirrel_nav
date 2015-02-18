// RawLaser.h --- 
// 
// Filename: RawLaser.h
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

#ifndef SQUIRREL_LOCALIZER_RAWLASER_H_
#define SQUIRREL_LOCALIZER_RAWLASER_H_

#include <string>
#include <vector>

#include "squirrel_localizer/Geometry.h"

struct LaserParameters {
  int type;
  float startAngle;
  float fov;
  float angularResolution;
  float accuracy;
  int remissionMode;
  float maxRange;
};

class RawLaser {
 public:
  // default constructor
  RawLaser();

  // copy constructor
  RawLaser(const RawLaser& l);

  // assignment operator
  RawLaser& operator=(const RawLaser& l);

  // destructor
  ~RawLaser();

  // set the values according to the entry
  void setValues(std::string logEntry);

  // 
  std::string toString();

  std::vector<Point> getScanPoints() const;
  std::vector<Point> getRemissionPoints() const;

  LaserParameters m_parameters;

  std::vector<float> m_ranges;
  std::vector<float> m_remissions;

  double m_timeStamp;
};

#endif /* SQUIRREL_LOCALIZER_RAWLASER_H_ */

//
// RawLaser.h ends here
