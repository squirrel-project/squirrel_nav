// BasicSensor.h --- 
// 
// Filename: BasicSensor.h
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

#ifndef SQUIRREL_LOCALIZER_BASIC_SENSOR_H__
#define SQUIRREL_LOCALIZER_BASIC_SENSOR_H__

#include <boost/random/mersenne_twister.hpp>

#include "squirrel_localizer/Transformation.h"

#include "squirrel_localizer/LocParameters.h"
#include "squirrel_localizer/LocParticle.h"

class BasicSensor
{
    
public:
  BasicSensor();
  virtual ~BasicSensor() {};

  virtual void set_params(SensorParameters *params);
  void filter_state(
      double                  metric_area,
      double                  angular_area,
      double                  linear_motion,
      bool                    active_localization
                    );

  bool bounded() const;
  bool localized() const;
  void setLocalized(bool state);

protected:
  virtual void handle_sensor_disagreement(ParticleVector &particles) = 0;
  virtual void resample_particles(ParticleVector &particles, int count) = 0;
  virtual void resampling_weights(
      ParticleVector         &particles,
      int                     count,
      double                 *resample_weight,
      double                 *mean_weight
                                  );
  ParticleVector::const_iterator best_particle(
      const ParticleVector &particles
                                               ) const;

protected:
  SensorParameters               *m_params;
  boost::mt19937                  m_rand_engine;
  int                             m_disagreements;
  double                          m_divergence_cum_distance;
  double                          m_convergence_cum_distance;
  bool                            m_bounded;
  bool                            m_localized;

};

#endif /* SQUIRREL_LOCALIZER_BASIC_SENSOR_H__ */

//
// BasicSensor.h ends here
