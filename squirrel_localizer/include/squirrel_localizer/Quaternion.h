// Quaternion.h --- 
// 
// Filename: Quaternion.h
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

#ifndef SQUIRREL_LOCALIZER_QUATERNION_H_
#define SQUIRREL_LOCALIZER_QUATERNION_H_

#include <limits>
#include <iostream>

#include <assert.h>

#include "squirrel_localizer/RotationMatrix.h"

using namespace std;

template <typename Base=double>
  struct _Quaternion : public _Vector<4, Base>
{
  static const int Angles=3;
  static const int Dimension=3;
  _Quaternion();
  _Quaternion(Base x, Base y, Base z, Base w);
  _Quaternion(const _RotationMatrix3<Base>& m);
  _Quaternion(const _Vector<3, Base>& vec);
  _Quaternion(Base roll, Base pitch, Base yaw);
  _Quaternion<Base>& operator*=(const _Quaternion& q);
  _Quaternion<Base>  operator* (const _Quaternion& q) const;
  template <typename Base2>
  _Quaternion(const _Quaternion<Base2>& other);  // Enable cast from quaternions with other, but compatible base
  _Vector<3, Base> operator*(const _Vector<3, Base>& v) const;
  inline _Quaternion<Base> inverse() const;
  inline _Vector<3, Base> angles() const;

  _RotationMatrix3<Base> rotationMatrix() const;
  _Quaternion<Base> normalized() const;
  
  //this function normalizes the quaternion and ensures thar w>0.
  _Quaternion<Base>& normalize();
  Base  angle() const;
  static inline _Quaternion<Base> slerp(const _Quaternion<Base>& from, const _Quaternion<Base>& to, Base lambda);
 protected:
  _Quaternion(const _Vector<4, Base>& v);
};

typedef _Quaternion<double> Quaternion;
typedef _Quaternion<float>  Quaternionf;

#include "squirrel_localizer/Quaternion.hpp"

#endif /* SQUIRREL_LOCALIZER_QUATERNION_H_ */

//
// Quaternion.h ends here
