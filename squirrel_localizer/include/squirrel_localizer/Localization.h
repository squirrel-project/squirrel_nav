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

#include <ros/io.h>

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

#include <geometry_msgs/PoseWithCovarianceStamped.h>

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

  Localization( void );
  
  void start_global( void );
  void start_rand_noise( void );
  void place_robot( const Transformation3&, double, bool usePoseTheta = false );

  void update_motion( const Transformation3&, const Transformation3& );

  bool update_laser( const std::vector<Vector3f>&, const Transformation3&, const Transformation3&, const double& );

  void particles( int );

  int particles( void ) const;
  void getParticles( ParticleVector*&, bool old=true );

  void set_map( AISNavigation::FloatMap* );
  AISNavigation::FloatMap* get_d_map( void );

  Transformation3 best_pose( void ) const;
  
  inline Transformation3& getMean( void )
  {
    return m_mean;
  };

  inline Transformation3 getMeanInterp( void )
  {
    return m_mean * m_motion_model.cummulative_motion();
  };

  inline void getCovariance( geometry_msgs::PoseWithCovarianceStamped& pose_msg )
  {
    pose_msg.pose.covariance[0] = m_covariance[0][0];
    pose_msg.pose.covariance[1] = m_covariance[0][1];
    pose_msg.pose.covariance[2] = m_covariance[0][2];
    pose_msg.pose.covariance[5] = m_covariance[0][5];

    pose_msg.pose.covariance[6] = m_covariance[1][0];
    pose_msg.pose.covariance[7] = m_covariance[1][1];
    pose_msg.pose.covariance[8] = m_covariance[1][2];
    pose_msg.pose.covariance[11] = m_covariance[1][5];

    pose_msg.pose.covariance[12] = m_covariance[2][0];
    pose_msg.pose.covariance[13] = m_covariance[2][1];
    pose_msg.pose.covariance[14] = m_covariance[2][2];
    pose_msg.pose.covariance[17] = m_covariance[2][5];

    pose_msg.pose.covariance[30] = m_covariance[5][0];
    pose_msg.pose.covariance[31] = m_covariance[5][1];
    pose_msg.pose.covariance[32] = m_covariance[5][2];
    pose_msg.pose.covariance[35] = m_covariance[5][5];
  };
  
  inline void set_localization_state( LocalizerState& state )
  {
    m_state = state;
  }

  inline bool& isInitialized( void )
  {
    return m_state.initialized;
  }
  
  void set_initialized( bool );
  
  inline bool& isLocalized( void )
  {
    return m_state.localized;
  }

  void set_localized( bool );
  
  bool has_converged( Transformation3&, Matrix6&, bool& ) const;

  void propagate_particles( void );

  void update_filter( BasicSensor&, const Transformation3&, const Transformation3&, const double& );
  void update_state( const Transformation3&, const Transformation3&, const double& );
  void compute_mean( void );
  void compute_covariance( void );
  void determine_convergence( void );
  void perform_resampling( void );
  void check_disagreements( void);
  void reset_motion( void );
  double particle_weight_variance( void ) const;
  double particle_weight_neff( void ) const;
  Transformation3 random_new_pose( void );
  Transformation3 random_6d_pose( const Transformation3&, double, double );
	
  inline void setRandomSeed( uint32_t seed )
  {
    m_rand_engine.seed(seed); m_motion_model.setSeed(seed);
  }

  // Parameters
  LocalizerParameters m_localizer_params;
  LaserSensorParameters m_laser_sensor_params;
  MotionModelParameters m_motion_model_params;
  void reload_parameters( void );

protected:
  ParticleVector m_particles;
  ParticleVector m_particles_old;
  int m_disagreements;

  SimpleMotionModel m_motion_model;
  LaserSensor m_laser;

  AISNavigation::FloatMap *m_map;

  Transformation3 m_mean;
  Matrix6 m_covariance;
  bool m_bounded;
  double m_convergence_cum_distance;
  double m_divergence_cum_distance;

  boost::mt19937 m_rand_engine;

  bool m_active_localization;
  int m_active_localization_laser_count;
  LocalizerState m_state;
};

#endif /* __LOCALIZER_H__ */

//
// Localization.h ends here
