// BasicSensor.cpp --- 
// 
// Filename: BasicSensor.cpp
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:46:22 2015 (+0100)
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


#include "squirrel_localizer/BasicSensor.h"

BasicSensor::BasicSensor()
    :   m_params(0)
    , m_rand_engine(time(0))
    , m_disagreements(0)
    , m_divergence_cum_distance(0.0)
    , m_convergence_cum_distance(0.0)
    , m_bounded(false)
    , m_localized(false)
{}

void BasicSensor::set_params(SensorParameters *params)
{
  m_params = params;
}

void BasicSensor::filter_state(
    double                          metric_area,
    double                          angular_area,
    double                          linear_motion,
    bool                            active_localization = false
                               )
{
  m_bounded = metric_area < m_params->convergenceRadius &&
      angular_area < m_params->convergenceAngle;
		
  //     cerr << "Metric: " << metric_area << ", Angular: " << angular_area << ", motion: " << linear_motion << ", bounded: " << m_bounded << endl;

  // Perform the state transition according to the particle status
  if(m_localized) {
    if(!m_bounded) {
      m_divergence_cum_distance += linear_motion;
      if(m_divergence_cum_distance > m_params->divergenceDistance) {
        m_divergence_cum_distance   = 0.0;
        m_convergence_cum_distance  = 0.0;
        m_localized                 = false;
      }
      if(active_localization)
        m_localized = false;
    } else {
      m_divergence_cum_distance = 0.0;
    } 
  } else {
    if(m_bounded) {
      m_convergence_cum_distance += linear_motion;
      if(m_convergence_cum_distance > m_params->convergenceDistance) {
        m_divergence_cum_distance   = 0.0;
        m_convergence_cum_distance  = 0.0;
        m_localized                 = true;
      }
      if(active_localization)
        m_localized = true;
    } else {
      m_convergence_cum_distance = 0.0;
      m_localized                 = false;
    } 
  }
}

bool BasicSensor::bounded() const
{
  return m_bounded;
}

bool BasicSensor::localized() const
{
  return m_localized;
}

void BasicSensor::setLocalized(bool state) 
{
  m_bounded = state;
  m_localized = state;
}

void BasicSensor::resampling_weights(
    ParticleVector                 &particles,
    int                             count,
    double                         *resample_weight,
    double                         *mean_weight
                                     )
{
  vector<double> weights;
  *mean_weight = 0.0;
  double log_max = *max_element(particles.begin(), particles.end());
  for(ParticleVector::const_iterator it=particles.begin(); it!=particles.end(); ++it) {
    weights.push_back(exp(it->weight - log_max));
    *mean_weight += exp(it->weight - log_max);
  }
  *mean_weight /= weights.size();
    
  sort(weights.begin(), weights.end());
  *resample_weight = weights[count];
}

/**
 * \brief Returns a const iterator to the particle with the highest weight.
 *
 * \param particles vector with all particles
 * \return interator to the best particle
 */
ParticleVector::const_iterator BasicSensor::best_particle(
    const ParticleVector           &particles
                                                          ) const
{
  ParticleVector::const_iterator best = particles.begin();
  for(ParticleVector::const_iterator it=particles.begin(); it!=particles.end(); ++it) {
    if(best->weight < it->weight) {
      best = it;
    }
  }
  return best;
}

// 
// BasicSensor.cpp ends here
