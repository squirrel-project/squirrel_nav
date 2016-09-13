// RobotLaser.h --- 
// 
// Filename: RobotLaser.h
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

#ifndef SQUIRREL_LOCALIZER_ROBOTLASER_H_
#define SQUIRREL_LOCALIZER_ROBOTLASER_H_

#include <string>
#include <vector>

#include "squirrel_localizer/RawLaser.h"
#include "squirrel_localizer/Geometry.h"

class RobotLaser: public RawLaser {
 public:
  // default constructor
  RobotLaser();

  // copy constructor
  RobotLaser(const RobotLaser& l);

  // assignment operator
  RobotLaser& operator=(const RobotLaser& l);

  // destructor
  ~RobotLaser();

  // set the values according to the entry
  void setValues(std::string logEntry);

  // 
  std::string toString();

  std::vector<Point> getRemissionPoints() const;
  std::vector<Point> getScanPoints() const;
  std::vector<Point> getMeasuredPoints() const;

  Pose m_laserPose;
  Pose m_robotPose;

  double m_laserTv; 
  double m_laserRv; 
  double m_forwardSafetyDist; 
  double m_sideSaftyDist;
  double m_turnAxis;

};

typedef struct std::vector<RobotLaser> RobotLaserVector;

#endif /* SQUIRREL_LOCALIZER_ROBOTLASER_H_ */

//
// RobotLaser.h ends here
