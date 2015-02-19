// LserSensor2D.cpp --- 
// 
// Filename: LaserSensor2D.cpp
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:48:18 2015 (+0100)
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


#include "squirrel_localizer/LaserSensor2D.h"

#include <ros/ros.h>

#include <string>
#include <fstream>
#include <sstream>

using namespace std;


/**
 * \brief Constructor for a LaserSensor object.
 */
LaserSensor::LaserSensor(double maxDistance, double minOccupancy, bool markUnknown)
    :   m_map(0)
    , m_world_endpoints()
    , m_maxDistance(maxDistance)
    , m_minOccupancy(minOccupancy)
    , m_markUnknown(markUnknown)
    ,m_distanceMapComputed(false)
{
  m_distanceMap = new AISNavigation::FloatMap;
}

/**
 * \brief Computes the weights for all particles based on the given scan.
 *
 * \param particles set of particles
 * \param endpoints laser scan endpoints
 */
void LaserSensor::compute_weights(
    ParticleVector                 &particles,
    const vector<Vector3f>         &endpoints
                                  )
{
  filter_measurement(endpoints, m_map->resolution());
  double min = 1e9;
  double max = -1e9;
  for(ParticleVector::iterator it=particles.begin(); it!=particles.end(); ++it) {
    double val = particle_weight(*it);
    //         it->weight += log(val); //m_params->weight * //FIXME BUG?
    it->weight += val; //m_params->weight * 
    min = val < min ? val : min;
    max = val > max ? val : max;
  }

  //cerr << "L: " << min << " " << max << endl;
}

/**
 * \brief Determines if a sensor update has to be performed.
 */
bool LaserSensor::perform_update(double linear_motion, double angular_motion) const
{
  if(linear_motion  < m_params->linearUpdate  && 
     angular_motion < m_params->angularUpdate
     ) {
    return false;
  }
  return true;
}

/**
 * \brief Sets the current graph map.
 *
 * \param graph graph map to use
 */
void LaserSensor::set_map(AISNavigation::FloatMap *map)
{
  std::string ns = ros::this_node::getNamespace();
  std::string nn = ros::this_node::getName();
  ROS_INFO("%s/%s: Setting the map", ns.c_str(), nn.c_str());
  m_map = map;
  m_distanceMapComputed = false;
}

void LaserSensor::handle_sensor_disagreement(ParticleVector &particles)
{
  (void)particles; // Get rid of compiler warning
  cerr << "LaserSensor::handle_sensor_disagreement not implemented" << endl;
}

void LaserSensor::resample_particles(ParticleVector &particles, int count)
{
  (void)particles; // Get rid of compiler warning
  (void)count; // Get rid of compiler warning
  cerr << "LaserSensor::resample_particles not implemented" << endl;
}


/**
 * \brief Filters a measurement by removing invalid values.
 *
 * FIXME: (Lionel) only considers 2D information of the endpoints, and
 *        not the 3D data.  
 *
 * \param endpoints enpoints of the laser scan
 * \param resolution gridmap resolution
 */
void LaserSensor::filter_measurement(
    const vector<Vector3f>         &endpoints,
    double                          resolution
                                     )
{
  Vector2 last_valid(
      numeric_limits<double>::max(), numeric_limits<double>::max()
                     );
  LaserSensorParameters *params = dynamic_cast<LaserSensorParameters*>(m_params);
  assert(params);
  double point_density2 = resolution * resolution *
      params->observationPointDensity;
  double max_beamrange2 = params->maxLocalizationRange *
      params->maxLocalizationRange;

  // Save valid endpoints of the beams in the global map coordinate system
  m_world_endpoints.clear();

  for(vector<Vector3f>::const_iterator it=endpoints.begin(); it!=endpoints.end(); ++it) {
    Vector2 current(it->x(), it->y());
    double  delta2  = (current - last_valid) * (current - last_valid);
    double  length2 = current * current;

    if(delta2 > point_density2 && length2 < max_beamrange2) {
      last_valid = current;
      m_world_endpoints.push_back(current);
    }
  }
}

/**
 * \brief Computes the weight of the particle according to the given map.
 *
 * \param p particle to compute the weight of
 * \return likelihood of the particle
 */
double LaserSensor::particle_weight(const LocalizerParticle &p)
{
  LaserSensorParameters *params = dynamic_cast<LaserSensorParameters*>(m_params);
  assert(params);
    
  AISNavigation::FloatMap *map = distance_map(p);
  Vector6 pose = p.pose.toVector();
  Transformation2 p_planar(Vector2(pose[0], pose[1]), pose[5]);

  // Get grid map position for each beam and accumulate the distance values
  double misalignment = 0.0;
  int missed_beams = 0;
  for(vector<Vector2>::const_iterator it=m_world_endpoints.begin();
      it!=m_world_endpoints.end();
      ++it
      ) {
    Vector2i cell_index = m_map->world2map(p_planar * (*it));
    Vector2 point = p_planar * (*it);
    if(map->isInside(cell_index)) {
      misalignment += map->cell(cell_index) * map->cell(cell_index);
    } else {
      misalignment += m_maxDistance * m_maxDistance;
      missed_beams++;
    }
  }

  // Compute likelihood in the range (0, 1] for the particle based on
  // the match quality of the scan
  double avg_error = misalignment / double(m_world_endpoints.size());
  double loglikelihood = min(0.0, -avg_error);
  //     loglikelihood /= m_map->m_maxDistance * m_map->m_maxDistance;


  //     loglikelihood = max(0.00001, likelihood);
  // Penalize beams ending too far away from an obstacle
  //likelihood *= 1.0 - (missed_beams / m_world_endpoints.size());
  //likelihood = max(0.00001, likelihood);

  return loglikelihood/(params->observationSigma*params->observationSigma);
}

/**
 * \brief Returns the distance map to use at the given position.
 *
 * \param pose current particle pose
 */
AISNavigation::FloatMap* LaserSensor::distance_map(const LocalizerParticle &p)
{
  if(!m_distanceMapComputed){
    std::string ns = ros::this_node::getNamespace();
    std::string nn = ros::this_node::getName();
    ROS_INFO("%s/%s: Computing the distance map.", ns.c_str(), nn.c_str());
    m_map->computeDistanceMap(*m_distanceMap, m_maxDistance, m_minOccupancy, m_markUnknown);
    m_distanceMapComputed = true;
  }
  return m_distanceMap;
}

// 
// LaserSensor2D.cpp ends here
