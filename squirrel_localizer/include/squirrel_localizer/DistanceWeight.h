// DistanceWeight.h --- 
// 
// Filename: DistanceWeight.h
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

#ifndef SQUIRREL_LOCALIZER_DISTANCE_WEIGHT_H__
#define SQUIRREL_LOCALIZER_DISTANCE_WEIGHT_H__

class DistanceWeight
{
public:
  virtual ~DistanceWeight();

  virtual double weight(double dist) const;
};

class LevyDistance : public DistanceWeight
{
public:
  LevyDistance(double gamma);

  double weight(double dist) const;

private:
  double                          m_gamma;
  double                          m_limit;
  double                          m_maxvalue;
};

class GaussianDistance : public DistanceWeight
{
public:
  GaussianDistance(double sigma);

  double weight(double dist) const;

private:
  double                          m_sigma;
  double                          m_prefactor;
  double                          m_sigma_square;
};

class ExponentialDistance : public DistanceWeight
{
public:
  double weight(double dist) const;
};

#endif /* SQUIRREL_LOCALIZER_DISTANCE_WEIGHT_H_ */

//
// DistanceWeight.h ends here
