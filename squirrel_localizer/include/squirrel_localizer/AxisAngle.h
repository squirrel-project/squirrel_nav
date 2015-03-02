// AxisAngle.h --- 
// 
// Filename: AxisAngle.h
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:16:09 2015 (+0100)
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

#ifndef SQUIRREL_LOCALIZER_AXISANGLE_H_
#define SQUIRREL_LOCALIZER_AXISANGLE_H_

#include "squirrel_localizer/Quaternion.h"

template <typename Base=double>
struct _AxisAngle : public _Vector<3, Base>
{
  static const int Dimension=3;
  static const int Angles=3;
  _AxisAngle();
  _AxisAngle(const _Quaternion<Base>& q);
  _AxisAngle(const _RotationMatrix3<Base>& m);
  _AxisAngle(const _Vector<3, Base>& vec);
  _AxisAngle(Base roll, Base pitch, Base yaw);
  _AxisAngle(const _Vector<3, Base>& axis, Base angle);
  _AxisAngle<Base>& operator*=(const _AxisAngle& a);
  _AxisAngle<Base>  operator* (const _AxisAngle& a) const;
  _Vector<3, Base> operator*(const _Vector<3, Base>& v) const;
  inline _AxisAngle<Base> inverse() const;
  inline _Vector<3, Base> angles() const;
  _RotationMatrix3<Base> rotationMatrix() const;
  _Quaternion<Base> quaternion() const;
};

typedef _AxisAngle<double> AxisAngle;
typedef _AxisAngle<float> AxisAnglef;

template <typename Base>
_AxisAngle<Base>::_AxisAngle():_Vector<3, Base> (Base(0.),Base(0.),Base(0.)) {}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(const _Quaternion<Base>& q) {
  Base angle=q.angle();
  Base imNorm = sqrt(q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
  if (imNorm < std::numeric_limits<Base>::min()){
    this->x()=Base(0.);
    this->y()=Base(0.);
    this->z()=Base(0.);
  } else {
    Base alpha=angle/imNorm;
    this->x()=q.x()*alpha;
    this->y()=q.y()*alpha;
    this->z()=q.z()*alpha;
  }
}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(const _RotationMatrix3<Base>& m) {
  *this=_Quaternion<Base>(m);
}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(const _Vector<3, Base>& vec) {
  *this=_Quaternion<Base>(vec);
}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(Base roll, Base pitch, Base yaw){
  *this=_AxisAngle(_Quaternion<Base>(roll, pitch, yaw));
}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(const _Vector<3, Base>& axis, Base angle){
  *this = axis.normalized() * angle;
}

template <typename Base>
_AxisAngle<Base>& _AxisAngle<Base>::operator*=(const _AxisAngle& a){
  *this=(*this)*a;
  return *this;
}
  
  
template <typename Base>
_AxisAngle<Base>  _AxisAngle<Base>::operator* (const _AxisAngle& a) const{
  return _AxisAngle<Base>(quaternion()*a.quaternion());
}
  
template <typename Base>
_Vector<3, Base> _AxisAngle<Base>::operator*(const _Vector<3, Base>& v) const {
  return rotationMatrix()*v;
}

template <typename Base>
inline _AxisAngle<Base> _AxisAngle<Base>::inverse() const {
  _AxisAngle a(*this);
  a._Vector<3, Base>::operator*=(Base(-1.));
  return a;
}
  
template <typename Base>
inline _Vector<3, Base> _AxisAngle<Base>::angles() const {
  return quaternion().angles();
}

template <typename Base>
_RotationMatrix3<Base> _AxisAngle<Base>::rotationMatrix() const{
  return quaternion().rotationMatrix();
}

template <typename Base>
_Quaternion<Base> _AxisAngle<Base>::quaternion() const {
  Base n=this->norm();
  if (n<=0){
    return _Quaternion<Base>();
  }
  Base s=sin(n*Base(.5))/n;
  Base c=cos(n*Base(.5));
  return _Quaternion<Base> (this->x()*s, this->y()*s, this->z()*s, c);
}

#endif /* SQUIRREL_LOCALIZER_AXISANGLE_H_ */

// 
// AxisAngle.h ends here
