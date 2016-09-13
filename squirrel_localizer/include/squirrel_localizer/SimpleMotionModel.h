// SimpleMotionModel.h --- 
// 
// Filename: SimpleMotionModel.h
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

#ifndef SQUIRREL_LOCALIZER_SIMPLEMOTIONMODEL_H_
#define SQUIRREL_LOCALIZER_SIMPLEMOTIONMODEL_H_

#include <fstream>

#include "squirrel_localizer/Transformation.h"
#include "squirrel_localizer/Vector_n.h"

#include "squirrel_localizer/BasicMotion.h"
#include "squirrel_localizer/LocParameters.h"

#include <boost/random/uniform_01.hpp>
#include <boost/random.hpp>

class SimpleMotionModel : public BasicMotion
{
public:
SimpleMotionModel(): gen(0), rng(gen, distribution) { }
  virtual ~SimpleMotionModel() { }
	
  void set_params(MotionModelParameters* motion_model_params);

  void prepare_sampling(const Transformation3 &motion, double delta);
  Transformation3 sample_pose_change(const Transformation3 &motion) const;

  bool perform_update() const;
	
  inline void setSeed(int32_t s) {gen.seed(s);}

private:
  double triangular_sample(double width, double mean=0.0) const;

private:
  MotionModelParameters*          m_params;
	
  mutable boost::mt19937 gen;
  mutable boost::uniform_01<double> distribution;
  mutable boost::variate_generator<boost::mt19937, boost::uniform_01<double> > rng;
	
  // Width of the triangular distribution
  double                          m_wx;
  double                          m_wy;
  double                          m_wtheta;
};

#endif /* SQUIRREL_LOCALIZER_SIMPLEMOTIONMODEL_H_ */

//
// SimpleMotionModel.h ends here
