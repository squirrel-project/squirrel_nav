// ForwardDeclarations.h --- 
// 
// Filename: ForwardDeclarations.h
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

#ifndef SQUIRREL_LOCALIZER_FORWARD_DECLARATIONS_H_
#define SQUIRREL_LOCALIZER_FORWARD_DECLARATIONS_H_

// Vector
template <int N, typename Base>
struct _Vector;
typedef _Vector<0,double> VectorX;
typedef _Vector<0,float>  VectorXf;
typedef _Vector<2,double> Vector2;
typedef _Vector<2,int>    Vector2i;
typedef _Vector<2,float>  Vector2f;
typedef _Vector<3,double> Vector3;
typedef _Vector<3,float>  Vector3f;
typedef _Vector<6,double> Vector6;
typedef _Vector<6,float>  Vector6f;

// Matrix
template <int Rows, int Cols, typename Base>
struct _Matrix;
typedef _Matrix<0, 0, double> MatrixX;
typedef _Matrix<0, 0, float>  MatrixXf;
typedef _Matrix<2, 2, double> Matrix2;
typedef _Matrix<2, 2, float>  Matrix2f;
typedef _Matrix<3, 3, double> Matrix3;
typedef _Matrix<3, 3, float>  Matrix3f;
typedef _Matrix<6, 6, double> Matrix6;
typedef _Matrix<6, 6, float>  Matrix6f;

#endif  /* SQUIRREL_LOCALIZER_FORWARD_DECLARATION_H_ */

//
// ForwardDeclarations.h ends here
