// Geometry.h --- 
// 
// Filename: Geometry.h
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

#ifndef SQUIRREL_LOCALIZER_GEOMETRY_H_
#define SQUIRREL_LOCALIZER_GEOMETRY_H_

#include <vector>

class IntPair {
 public:
  IntPair();
  IntPair(int i, int j);

  IntPair& operator=(const IntPair& ip);
  
  bool operator==(const IntPair& other) const;
  bool operator!=(const IntPair& other) const;
  
  int i;
  int j;
};

typedef struct std::vector<IntPair> IntPairVector;

class ValuedIntPair {
 public:
  ValuedIntPair();
  ValuedIntPair(int i, int j);
  ValuedIntPair(int i, int j, double v);
  ValuedIntPair(const ValuedIntPair& ip);  

  ValuedIntPair& operator=(const ValuedIntPair& ip);
  
  bool operator==(const ValuedIntPair& other) const;
  bool operator!=(const ValuedIntPair& other) const;
  bool operator< (const ValuedIntPair& other) const;
  
  int i;
  int j;
  double value;
};

class Pose {
 public:
  Pose();
  Pose(double x, double y, double theta);
  Pose(const Pose& p);  

  Pose& operator=(const Pose& p);

  double x;
  double y;
  double theta;
  double theta_x;
  double theta_y;
};

typedef struct std::vector<Pose> PoseVector;

class Movement {
 public:
  Movement();
  Movement(double forward, double sideward, double rotation);
  Movement(const Movement& m);  

  Movement& operator=(const Movement& m);

  double getLinearDistance();
  double getAngularDistance();

  double forward;
  double sideward;
  double rotation;
};

struct Point {
  double x;
  double y;

Point() : x(0.0), y(0.0) {}
Point(double x_, double y_) : x(x_), y(y_) {}
};

double normalizeAngle(double theta);

double orientationDiff(double start, double end);

double computeDistance(Pose start, Pose end);

double computeDistance(Point start, Point end);

double computeSquaredDistance(Point start, Point end);

double computeFactorial(double n);

Movement computeMovement(Pose start, Pose end);

Pose applyMovement(Pose p, Movement m);

std::vector<Point> sparsify(const std::vector<Point>& points, double distance);


#endif /* SQUIRREL_LOCALIZER_GEOMETRY_H_ */

//
// Geometry.h ends here
