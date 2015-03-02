// LocParticle.h --- 
// 
// Filename: LocParticle.h
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

#ifndef SQUIRREL_LOCALIZER_LOCPARTICLE_H_
#define SQUIRREL_LOCALIZER_LOCPARTICLE_H_

#include "squirrel_localizer/Transformation.h"

#include <vector>

class LocalizerParticle
{
 public:
  //! Weight of the particle
  double                          weight;
  //! 3D pose represented by the particle
  Transformation3                 pose;
  //! ID of the currently used node
  int                             node_id;

  inline LocalizerParticle(){
    Vector6 new_pose;
    new_pose.fill(0.0);
    pose = Transformation3::fromVector(new_pose);
    weight = 0.;
    node_id = -1;
  }

  inline operator std::vector<float>() const {
    std::vector<float> result(6,0.);
    result[0]=pose[0];
    result[1]=pose[1];
    result[2]=pose[2];
    result[3]=pose[3];
    result[4]=pose[4];
    result[5]=pose[5];
    return result;
  }

  /**
   * \brief Returns the weight of the particle.
   *
   * \return weight of the particle
   */
  inline operator double() const
  {
    return weight;
  }

  /**
   * \brief Sets the weight of the particle and returns it.
   *
   * \param w new particle weight
   * \return reference to the particle
   */
  inline LocalizerParticle& operator=(double w)
  {
    weight = w;
    return *this;
  }
};

typedef std::vector<LocalizerParticle> ParticleVector;
    
#endif /* SQUIRREL_LOCALIZER_LOCPARTICLE_HPP_ */

//
// LocParticle.h ends here
