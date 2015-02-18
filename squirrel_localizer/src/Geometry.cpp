// Geometry.cpp --- 
// 
// Filename: Geometry.cpp
// Description: Geometry tools for motion update
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 12:06:38 2015 (+0100)
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

#include <math.h>

#include "squirrel_localizer/Geometry.h"

IntPair::IntPair() {
  i = 0;
  j = 0;
}

IntPair::IntPair(int i_, int j_) {
  i = i_;
  j = j_;
}

IntPair& IntPair::operator=(const IntPair& ip) {
  if (this != &ip) {
    i = ip.i;
    j = ip.j;
  }

  return *this;
}

bool IntPair::operator==(const IntPair& other) const {
  return i == other.i && j == other.j;
}

bool IntPair::operator!=(const IntPair& other) const {
  return i != other.i || j != other.j;
}

ValuedIntPair::ValuedIntPair() {
  i = 0;
  j = 0;
  value = 0.0;
}

ValuedIntPair::ValuedIntPair(int i_, int j_) {
  i = i_;
  j = j_;
  value = 0.0;
}

ValuedIntPair::ValuedIntPair(int i_, int j_, double v) {
  i = i_;
  j = j_;
  value = v;
}

ValuedIntPair::ValuedIntPair(const ValuedIntPair& vip) {
  i = vip.i;
  j = vip.j;
  value = vip.value;
}

ValuedIntPair& ValuedIntPair::operator=(const ValuedIntPair& vip) {
  if (this != &vip) {
    i = vip.i;
    j = vip.j;
    value = vip.value;
  }

  return *this;
}

bool ValuedIntPair::operator==(const ValuedIntPair& other) const {
  return i == other.i && j == other.j;
}

bool ValuedIntPair::operator!=(const ValuedIntPair& other) const {
  return i != other.i || j != other.j;
}

bool ValuedIntPair::operator< (const ValuedIntPair& other) const {
  return value < other.value;
}

Pose::Pose() {
  x = 0.0;
  y = 0.0;
  theta = 0.0;
  theta_x = 0.0;
  theta_y = 0.0;
}

Pose::Pose(double x_, double y_, double theta_) {
  x = x_;
  y = y_;
  theta = theta_;
}

Pose::Pose(const Pose& p) {
  x = p.x;
  y = p.y;
  theta = p.theta;
  theta_x = p.theta_x;
  theta_y = p.theta_y;
}

Pose& Pose::operator=(const Pose& p) {
  if (this != &p) {
    x = p.x;
    y = p.y;
    theta = p.theta;
    theta_x = p.theta_x;
    theta_y = p.theta_y;
  }

  return *this;
}

Movement::Movement() {
  forward = 0.0;
  sideward = 0.0;
  rotation = 0.0;
}

Movement::Movement(double f, double s, double r) {
  forward = f;
  sideward = s;
  rotation = r;
}

Movement::Movement(const Movement& m) {
  forward = m.forward;
  sideward = m.sideward;
  rotation = m.rotation;
}

Movement& Movement::operator=(const Movement& m) {
  if (this != &m) {
    forward = m.forward;
    sideward = m.sideward;
    rotation = m.rotation;
  }

  return *this;
}

double Movement::getLinearDistance() {
  return sqrt(forward*forward + sideward*sideward);
}

double Movement::getAngularDistance() {
  return fabs(rotation);
}

double normalizeAngle(double theta) {
  double multiplier;
  
  if (theta >= -M_PI && theta < M_PI)
    return theta;
  
  multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;
  
  return theta;
  
}

double orientationDiff(double start, double end) {
  double diff = normalizeAngle( start ) - normalizeAngle( end );
  if (diff <- M_PI) {
    return(2*M_PI + diff);
  } else if (diff > M_PI) {
    return(-2*M_PI + diff);
  } else {
    return(diff);
  }
}

double computeDistance(Pose start, Pose end) {
  return sqrt((start.x - end.x)*(start.x - end.x) + (start.y - end.y)*(start.y - end.y));
}

double computeDistance(Point start, Point end) {
  return sqrt((start.x - end.x)*(start.x - end.x) + (start.y - end.y)*(start.y - end.y));
}

double computeSquaredDistance(Point start, Point end) {
  return (start.x - end.x)*(start.x - end.x) + (start.y - end.y)*(start.y - end.y);
}

double computeFactorial(double n) {
  double fact = 1.0;
  
  while ( n > 1) {
    fact = fact * n;
    n = n - 1;
  }

  return fact;
}

Movement computeMovement(Pose start, Pose end) {
  Movement m;

  m.forward =
    + (end.y - start.y) * sin(start.theta)
    + (end.x - start.x) * cos(start.theta);

  if (fabs(m.forward) < 0.001)
    m.forward = 0.0;

  m.sideward =
    - (end.y - start.y) * cos(start.theta)
    + (end.x - start.x) * sin(start.theta);

  if (fabs(m.sideward) < 0.001)
    m.sideward = 0.0;

  m.rotation = orientationDiff( end.theta, start.theta );

  if (fabs(m.rotation) < 0.001)
    m.rotation = 0.0;

  return m;
}

Pose applyMovement(Pose p, Movement m) {
  Pose end;
  if (( m.forward == 0.0 ) 
      && ( m.sideward == 0.0 ) 
      && ( m.rotation == 0.0))
    return (p);
  
  end.x = p.x + 
    cos(p.theta) * m.forward +
    sin(p.theta) * m.sideward;

  end.y = p.y +
    sin(p.theta) * m.forward -
    cos(p.theta) * m.sideward;

  end.theta = normalizeAngle( p.theta + m.rotation );

  return end;
}

std::vector<Point> sparsify(const std::vector<Point>& points, double distance){
  std::vector<Point> sp;
  
  if (points.size() == 0)
    return sp;

  sp.push_back(points[0]);
  Point lastPoint = points[0];

  for (unsigned int i = 1; i < points.size(); i++){
    
    double squaredDist = computeSquaredDistance(lastPoint, points[i]);
    
    if (squaredDist > distance){
      sp.push_back(points[i]);
      lastPoint = points[i];
    }
  }
  
  return sp;
}

// 
// Geometry.cpp ends here
