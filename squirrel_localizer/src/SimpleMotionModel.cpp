// SimpleMotionModel.cpp --- 
// 
// Filename: SimpleMotionModel.cpp
// Description: Simple motion model for particle filter 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:40:51 2015 (+0100)
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

#include "squirrel_localizer/SimpleMotionModel.h"

using namespace std;

void SimpleMotionModel::set_params(MotionModelParameters* motion_model_params)
{
  m_params = motion_model_params;
}

/**
 * \brief Computes values of the distribution to sample new poses from.
 *
 * \param motion motion performed since last update
 * \param delta time delta
 */
void SimpleMotionModel::prepare_sampling(
    const Transformation3          &motion,
    double                          delta
                                         )
{
  Vector6 motion6d = motion.toVector();
  m_wx        = m_params->ff * fabs(motion6d[0]) +
      m_params->fs * fabs(motion6d[1]) +
      m_params->fr * fabs(motion6d[5]) +
      m_params->sx * delta;
  m_wy        = m_params->fs * fabs(motion6d[0]) +
      m_params->ss * fabs(motion6d[1]) +
      m_params->sr * fabs(motion6d[5]) +
      m_params->sy * delta;
  m_wtheta    = m_params->fr * fabs(motion6d[0]) +
      m_params->sr * fabs(motion6d[1]) +
      m_params->rr * fabs(motion6d[5]) +
      m_params->sth * delta;
  m_wx       *= m_params->magnitude;
  m_wy       *= m_params->magnitude;
  m_wtheta   *= m_params->magnitude;
}

/**
 * \brief Changes to pose of according to the motion model.
 *
 * \param motion base motion before noise
 */
Transformation3 SimpleMotionModel::sample_pose_change(const Transformation3 &motion) const
{
  Vector6 motion6d = motion.toVector();
  Transformation3 pose_change(
      Vector3(
          motion6d[0] + triangular_sample(m_wx, 0.0),
          motion6d[1] + triangular_sample(m_wy, 0.0),
          0.0
              ),
      Quaternion(
          0.0,
          0.0,
          motion6d[5] + triangular_sample(m_wtheta, 0.0)
                 )
                              );
  return pose_change; 
}

bool SimpleMotionModel::perform_update() const
{
  return true;
}

/**
 * \brief Draw from a triangular distribution.
 *
 * \param width width of the distribution
 * \param mean mean of the distribution
 */
double SimpleMotionModel::triangular_sample(double width, double mean) const
{
  double u = rng();
  if(u <= 0.5) {
    u = -0.5 + sqrt(u * 0.5);
  } else {
    u = 0.5 - sqrt( (1-u) * 0.5);
  }
  return (u * 2 * width) + mean;
}


// 
// SimpleMotionModel.cpp ends here
