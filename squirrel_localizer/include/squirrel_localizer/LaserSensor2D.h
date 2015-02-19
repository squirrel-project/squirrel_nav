// LaserSensor2D.h --- 
// 
// Filename: LaserSensor2D.h
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

#ifndef SQUIRREL_LOCALIZER_LASERSENSOR2D_H_
#define SQUIRREL_LOCALIZER_LASERSENSOR2D_H_

#include "squirrel_localizer/Vector_n.h"
#include "squirrel_localizer/FloatMap.h"

#include "squirrel_localizer/BasicSensor.h"
#include "squirrel_localizer/LocParticle.h"
#include "squirrel_localizer/LocParameters.h"

class LaserSensor : public BasicSensor
{
public:
  LaserSensor(double maxDistance, double minOccupancy, bool markUnknown);
	
  virtual ~LaserSensor() { }

  void compute_weights(
      ParticleVector                 &particles,
      const std::vector<Vector3f>    &endpoints
                       );
  bool perform_update(double linear_motion, double angular_motion) const;

  void set_map(AISNavigation::FloatMap *map);

  //HACK TODO NEED TO BE DONE BETTER... Maybe using friendship or a singleton for the distance map
  inline AISNavigation::FloatMap* distance_map() {LocalizerParticle p; return distance_map(p);}

protected:
  void handle_sensor_disagreement(ParticleVector &particles);
  void resample_particles(ParticleVector &particles, int count);

  //     private:
  void filter_measurement(
      const std::vector<Vector3f>    &endpoints,
      double                          resolution
                          );
  double particle_weight(const LocalizerParticle &p);

  AISNavigation::FloatMap* distance_map(const LocalizerParticle &p);

  //     private:
  AISNavigation::FloatMap        *m_map;
  AISNavigation::FloatMap        *m_distanceMap;
  std::vector<Vector2>            m_world_endpoints;
  double 													m_maxDistance;
  double 													m_minOccupancy;
  bool	 													m_markUnknown;
  bool 									m_distanceMapComputed;
};

#endif /* SQUIRREL_LOCALIZER_LASERSENSOR2D_H_ */

//
// LaserSensor2D.h ends here
