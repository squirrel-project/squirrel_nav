// Vector_n.h --- 
// 
// Filename: Vector_n.h
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

#ifndef SQUIRREL_LOCALIZER_VECTORN_H_
#define SQUIRREL_LOCALIZER_VECTORN_H_

#include <iostream>
#include <cmath>
#include <algorithm>

#include <assert.h>

#include "squirrel_localizer/ArrayAllocator.h"

/** @addtogroup math **/
//@{

template <int Rows, int Cols, typename Base>
  struct _Matrix;

template <int N, typename Base=double>
struct _Vector{
  typedef Base BaseType;
  typedef Base value_type; // compatibility with STL

  inline int size() const {return _allocator.size();}

  inline const Base& operator[](int i) const {return _allocator[i];}
  inline bool operator==(const _Vector<N, Base>& other) const;

  inline Base& operator[](int i) {return _allocator[i];}

  _Vector(int s=N): _allocator(s){}
  static const int TemplateSize=N;

  template <typename Base2>
    _Vector(const _Vector<N, Base2>& v);
  
  _Vector(Base v0, Base v1){_allocator[0]=v0; _allocator[1]=v1;}

  _Vector(Base v0, Base v1, Base v2) {_allocator[0]=v0; _allocator[1]=v1; _allocator[2]=v2;}

  _Vector(Base v0, Base v1, Base v2, Base v3) {_allocator[0]=v0; _allocator[1]=v1; _allocator[2]=v2; _allocator[3]=v3;}

  inline const Base& x()     const  {return _allocator[0];}

  inline const Base& y()     const  {return _allocator[1];}

  inline const Base& z()     const  {return _allocator[2];}

  inline const Base& w()     const  {return _allocator[3];}

  inline Base& x()            {return _allocator[0];}

  inline Base& y()            {return _allocator[1];}

  inline Base& z()            {return _allocator[2];}

  inline Base& w()            {return _allocator[3];}

  inline const Base& roll()   const  {if (size() == 6) return _allocator[3]; return _allocator[0];}

  inline const Base& pitch() const  {if (size() == 6) return _allocator[4]; return _allocator[1];}

  inline const Base& yaw()   const  {if (size() == 6) return _allocator[5]; return _allocator[2];}

  inline Base& roll()          {if (size() == 6) return _allocator[3]; return _allocator[0];}

  inline Base& pitch()        {if (size() == 6) return
 _allocator[4]; return _allocator[1];}

  inline Base& yaw()          {if (size() == 6) return _allocator[5]; return _allocator[2];}


  _Vector<N,Base> operator + (const _Vector<N,Base>& v) const;

  _Vector<N,Base>& operator += (const _Vector<N,Base>& v);

  _Vector<N,Base> operator - (const _Vector<N,Base>& v) const;

  _Vector<N,Base>& operator -= (const _Vector<N,Base>& v);

  Base operator *(const _Vector<N,Base>& v) const;

  _Vector<N,Base>& operator *= (Base c);

  _Vector<N,Base> operator * (Base c) const;

  Base squaredNorm() const;

  Base norm() const;

  void normalize();

  void fill(Base scalar);

  _Vector<N,Base> normalized() const;

  operator _Matrix<N, 1, Base>() const;

  _ArrayAllocator<N,Base> _allocator;

};

template <int N, typename Base>
  std::ostream& operator << (std::ostream& os, const _Vector<N, Base>& v);

template <int N, typename Base>
_Vector<N, Base> operator* (Base x, const _Vector<N, Base>& v);

typedef _Vector<2,int>   Vector2i;
typedef _Vector<2,float> Vector2f;
typedef _Vector<3,float> Vector3f;
typedef _Vector<6,float> Vector6f;


typedef _Vector<2,double> Vector2;
typedef _Vector<3,double> Vector3;
typedef _Vector<6,double> Vector6;

typedef _Vector<0,double> VectorX;
typedef _Vector<0,float>  VectorXf;

template <int N, typename Base>
  void st2dyn(_Vector<0,Base>& dest, const _Vector<N,Base> src){
  std2dyn(dest._allocator, src._allocator);
}

template <int N, typename Base>
  void dyn2st(_Vector<0,Base>& dest, const _Vector<N,Base> src){
  dyn2st(dest._allocator, src._alloator);
}

template <int N, typename Base>
template <typename Base2>
_Vector<N,Base>::_Vector(const _Vector<N, Base2>& v){
  for (int i=0; i<size(); i++)
    _allocator[i]=Base(0.);
  int s=std::min(size(),v.size());
  for (int i=0; i<s; i++)
    _allocator[i]=v._allocator[i];
}

template <int N, typename Base>
_Vector<N,Base> _Vector<N,Base>::operator + (const _Vector<N,Base>& v) const {
  _Vector<N,Base> ret(*this);
  ret+=v;
  return ret;
};

template <int N, typename Base>
_Vector<N,Base>& _Vector<N,Base>::operator += (const _Vector<N,Base>& v) {
  for (int i=0; i<size(); i++)
    _allocator[i]+=v._allocator[i];
  return *this;
};

template <int N, typename Base>
_Vector<N,Base> _Vector<N,Base>::operator - (const _Vector<N,Base>& v) const {
  _Vector<N,Base> ret(*this);
  ret-=v;
  return ret;
};

template <int N, typename Base>
_Vector<N,Base>& _Vector<N,Base>::operator -= (const _Vector<N,Base>& v) {
  for (int i=0; i<size(); i++)
    _allocator[i]-=v._allocator[i];
  return *this;
};

template <int N, typename Base>
Base _Vector<N,Base>::operator *(const _Vector<N,Base>& v) const{
  Base acc=Base(0);
  for (int i=0; i<size(); i++)
    acc+=_allocator[i]*v._allocator[i];
  return acc;
}

template <int N, typename Base>
_Vector<N,Base>& _Vector<N,Base>::operator *= (Base c){
  for (int i=0; i<size(); i++)
    _allocator[i]*=c;
  return *this;
}

template <int N, typename Base>
_Vector<N,Base> _Vector<N,Base>::operator * (Base c) const{
  _Vector<N,Base> ret(*this);
  ret*=c;
  return ret;
}

template <int N, typename Base>
bool _Vector<N,Base>::operator==(const _Vector<N, Base>& other) const {
  for (int i=0; i<size(); ++i)
    if ((*this)[i]!=other[i]) return false;
  return true;
}

template <int N, typename Base>
Base _Vector<N,Base>::squaredNorm() const{
  Base acc=Base(0);
  for (int i=0; i<size(); i++)
    acc+=_allocator[i]*_allocator[i];
  return acc;
}

template <int N, typename Base>
Base _Vector<N,Base>::norm() const{
  return sqrt(squaredNorm());
}


template <int N, typename Base>
void _Vector<N, Base>::normalize(){
  Base f=squaredNorm();
  if (f>Base(0))
    (*this)*=sqrt(Base(1.)/f);
  else {
    (*this)-=(*this);
    _allocator[0]=Base(1.);
  }
}

template <int N, typename Base>
_Vector<N,Base> _Vector<N, Base>::normalized() const{
  _Vector<N,Base> aux(*this);
  aux.normalize();
  return aux;
}

template <int N, typename Base>
std::ostream& operator << (std::ostream& os, const _Vector<N, Base>& v){
  for (int j=0; j<v.size(); j++){
    if (j > 0)
      os << " ";
    os << v[j];
  }
  return os;
}

template <int N, typename Base>
void _Vector<N, Base>::fill(Base scalar)
{
  for (int i=0; i<size(); i++)
    _allocator[i] = scalar;
}

template <int N, typename Base>
_Vector<N, Base> operator* (Base x, const _Vector<N, Base>& v)
{
  return v * x;
}

#endif /* SQUIRREL_LOCALIZER_VECTORN_H_ */

//
// Vector_n.h ends here
