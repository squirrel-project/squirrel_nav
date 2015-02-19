// LocParameters.h --- 
// 
// Filename: LocParaeters.h
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

#ifndef SQUIRREL_LOCALIZER_LOCALIZATION_PARAMETERS_H_
#define SQUIRREL_LOCALIZER_LOCALIZATION_PARAMETERS_H_

struct LocalizerState
{

  inline LocalizerState() {
    reference_timestamp = 0.0;
    localized = false;
    initialized = false;
  };
  double reference_timestamp;
  Transformation3 reference_odom;
  Transformation3 reference_pose;
  bool localized;
  mutable bool initialized;
};

struct LocalizerParameters 
{
  /** primary laser */
  string primary_laser_message_name;
  /** secondary laser */
  string secondary_laser_message_name;
  bool use_secondary_laser;
  /** pf filter */
  int particles;
  double minWeight;
  double resample_variance;
  /** map related */
  double max_distance;
  double min_occupancy;
  bool mark_unknown;
};

struct SensorParameters
{
  virtual ~SensorParameters() {};

  double weight;
  double linearUpdate;
  double angularUpdate;
  double convergenceRadius;
  double convergenceAngle;
  double convergenceDistance;
  double divergenceDistance;
  int disagreementThreshold;
};

struct GPSSensorParameters : public SensorParameters
{
  double hdopThreshold;
  double sigma;
};

struct IMUSensorParameters : public SensorParameters
{
  double sigma;
};

struct LaserSensorParameters : public SensorParameters
{
  double observationSigma;
  double observationPointDensity;
  double fullThreshold;
  double maxRange;
  double usableRange;
  double maxLocalizationRange;
  /** laser sensor model 2D/3D selection */
  bool                    use_laser_3d;
};

struct VisionSensorParameters : public SensorParameters
{
  double searchDistance;
};

struct MotionModelParameters
{
  // Noise parameters
  double ff;          //! forward forward
  double fs;          //! forward sideward
  double fr;          //! forward rotational
  double ss;          //! sideward sideward
  double sr;          //! sideward rotational
  double rr;          //! rotational rotational

  // Time dependent noise used for static convergence
  double sx;          //! x direction
  double sy;          //! y direction
  double sth;         //! orientation angle

  double magnitude;   //! scaling factor
};

#endif /* SQUIRREL_LOCALIZER_LOCALIZATION_PARAMETERS_H_ */

//
// LocParameters.h ends here
