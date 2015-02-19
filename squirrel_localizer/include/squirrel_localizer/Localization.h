// Localization.h --- 
// 
// Filename: Localization.h
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

#ifndef SQUIRREL_LOCALIZER_LOCALIZATION_H_
#define SQUIRREL_LOCALIZER_LOCALIZATION_H_

#include <vector>
#include <fstream>

#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>

#include "squirrel_localizer/PF.h"
#include "squirrel_localizer/Misc.h"
#include "squirrel_localizer/Vector_n.h"
#include "squirrel_localizer/Transformation.h"
#include "squirrel_localizer/Matrix_n.h"
#include "squirrel_localizer/LocParticle.h"
#include "squirrel_localizer/LocParameters.h"
#include "squirrel_localizer/SimpleMotionModel.h"
#include "squirrel_localizer/LaserSensor2D.h"
#include "squirrel_localizer/FloatMap.h"

class Localization
{
  typedef std::vector<Vector2> Vector2Vector;

public:
  //! Available sensor types
  enum SensorType
  {
    GPS,
    LASER,
    VISION
  };

public:
  Localization();

  void start_global();
  void start_rand_noise();
  void place_robot(const Transformation3 &pose, double sigma, bool usePoseTheta = false);

  /** motion update */
  void update_motion(const Transformation3 &motion);

  /** sensor updates */
  bool update_laser(const std::vector<Vector3f>& endpoints, const Transformation3& odomPose, const double& timestamp);

  /** particle stuff */
  void particles(int num);
  int particles() const;
  void getParticles(ParticleVector*& particels, bool old=true);

  /** dynamic map related */
  void set_map(AISNavigation::FloatMap *map);

  /** pose getter */
  Transformation3 best_pose() const;
  inline Transformation3& getMean(){return m_mean;};
  inline Transformation3  getMeanInterp(){return m_mean * m_motion_model.cummulative_motion();};

  /** filter state */
  inline void             set_localization_state(LocalizerState& state){ m_state = state; }
  inline bool&            isInitialized(){return m_state.initialized;}
  void             set_initialized(bool state);
  inline bool&            isLocalized(){return m_state.localized;}
  void             set_localized(bool state);
  bool             has_converged(Transformation3& mean, Matrix6& cov, bool& bounded) const;

  /** Scan matcher*/
  AISNavigation::FloatMap*  get_d_map();

  //     protected:
public:
  void propagate_particles();

  void update_filter(BasicSensor &sensor, const Transformation3& odomPose, const double& timestamp);
  void update_state(const Transformation3& odomPose, const double& timestamp);

  void compute_mean();
  void compute_covariance();
  void determine_convergence();
  void perform_resampling();
  void check_disagreements();
  void reset_motion();

  double particle_weight_variance() const;
  double particle_weight_neff() const;

  Transformation3 random_new_pose();
  Transformation3 random_6d_pose(
      const Transformation3  &mean,
      double                  sigma_trans,
      double                  sigma_rot
                                 );
	
  inline void setRandomSeed(uint32_t seed) { m_rand_engine.seed(seed); m_motion_model.setSeed(seed);}

public:
  // Parameters
  LocalizerParameters             m_localizer_params;
  LaserSensorParameters           m_laser_sensor_params;
  MotionModelParameters           m_motion_model_params;
  void reload_parameters();


protected:
  ParticleVector                  m_particles;
  ParticleVector                  m_particles_old;
  int                             m_disagreements;

  SimpleMotionModel               m_motion_model;
  LaserSensor                     m_laser;

  AISNavigation::FloatMap        *m_map;

  Transformation3                 m_mean;
  Matrix6                         m_covariance;
  bool                            m_bounded;
  double                          m_convergence_cum_distance;
  double                          m_divergence_cum_distance;

  boost::mt19937                  m_rand_engine;

  bool                            m_active_localization;
  int                             m_active_localization_laser_count;
  LocalizerState                  m_state;
};

#endif /* __LOCALIZER_H__ */

//
// Localization.h ends here
