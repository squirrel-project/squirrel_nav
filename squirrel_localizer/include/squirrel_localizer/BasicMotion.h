// BasicMotion.h --- 
// 
// Filename: BasicMotion.h
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:19:42 2015 (+0100)
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

#ifndef SQUIRREL_LOCALIZER_BASIC_MOTION_H_
#define SQUIRREL_LOCALIZER_BASIC_MOTION_H_

#include "squirrel_localizer/Transformation.h"

class BasicMotion
{
 public:
  virtual ~BasicMotion() {};

  virtual bool perform_update() const = 0;
  virtual void update_motion(const Transformation3 &motion);
  virtual void reset();

  double linear_motion() const;
  double angular_motion() const;
  Transformation3 cummulative_motion() const;

 private:
  double                          m_linear_motion;
  double                          m_angular_motion;
  Transformation3                 m_cummulative_motion;
};

#endif /* SQUIRREL_LOCALIZER_BASIC_MOTION_H_ */

// 
// BasicMotion.h ends here
