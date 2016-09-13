// Localization.cpp --- 
// 
// Filename: Localization.cpp
// Description: Localization node 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:12:25 2015 (+0100)
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


#include "squirrel_localizer/Localization.h"

#include <ctime>

using namespace AISNavigation;
using namespace std;

/**
 * \brief Creates a new Localizer instance.
 *
 * \param paramfile parameter configuration file
 * \param basename base name of the map files
 */
Localization::Localization() :
    m_disagreements(0),
    m_motion_model(),
    m_laser(m_localizer_params.max_distance, m_localizer_params.min_occupancy, m_localizer_params.mark_unknown),
    m_map(0),
    m_rand_engine(time(0)),
    m_active_localization(false),
    m_active_localization_laser_count(0)
{

  //     // Load parameters and pass them to the sensors
  //     loadParams();
  //     particles(m_localizer_params.particles);
  //     m_particles_old = ParticleVector(m_particles.size());
  // 
  //     m_motion_model.set_params(&m_motion_model_params);
  // 
  //     m_gps.set_params(&m_gps_sensor_params);
  //     m_laser.set_params(&m_laser_sensor_params);
  //     m_vision.set_params(&m_vision_sensor_params);
}


/**
 * \brief Samples particles randomly on the map.
 */
void Localization::start_global()
{
  particles(m_localizer_params.particles);

  for(ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    bool on_free_space = false;
    do {
      it->pose = random_new_pose();
      Vector2 wp(it->pose[0],it->pose[1]);
      Vector2i mp = m_map->world2map(wp);
      if ( m_map->isInside(mp) ) {
        on_free_space = (bool) !m_map->cell(mp);
      }
    } while ( !on_free_space );    
    it->weight = 0.;
  }
  reset_motion();
}

/**
 * \brief Starts random noise on particles localization mode.
 */
void Localization::start_rand_noise(){
  m_active_localization = true;
  m_active_localization_laser_count = 0;
}

/**
 * \brief Initializes the filter at the given position.
 *
 * Places all particles around the given position, scattering them with some
 * gaussian noise.
 *
 * \param pose initialization position
 * \param sigma scattering distance
 * \param usePoseTheta use the orientation of the pose as initialization
 */
void Localization::place_robot(const Transformation3 &pose, double sigma, bool usePoseTheta)
{
  boost::normal_distribution<> distribution_trans(0.0, sigma);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > gen_trans(m_rand_engine, distribution_trans);

  boost::normal_distribution<> distribution_rot(0.0, M_PI); 
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > gen_rot(m_rand_engine, distribution_rot);

  (void) gen_rot(); //Removes Warning
  //     int i = 0;
  particles(m_localizer_params.particles);
  for(ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    if(usePoseTheta){
      //Apllying theta from pose  
      Vector6 newPose = it->pose.toVector();
      newPose[5] = pose.toVector()[5];
      it->pose = Transformation3::fromVector(newPose);
	 
      Quaternion rotation(0.0, 0.0,  gen_rot() / 10.0 );
      Transformation3 delta(Vector3(gen_trans(), gen_trans(), 0.0), rotation);
      it->pose = pose * delta;
    } else {
      //Random sampling of theta 
      Quaternion rotation(0.0, 0.0, gen_rot());
      Transformation3 delta(Vector3(gen_trans(), gen_trans(), 0.0), rotation);
      it->pose = pose * delta;
      //           start_rand_noise();
    }
    //         it->weight = 1.0;
    it->weight = 0.;
  }

  reset_motion();
  compute_mean();
  compute_covariance();
}

/**
 * \brief Updates the motion model.
 *
 * \param motion recently performed motion
 */
void Localization::update_motion(const Transformation3 &odometry, const Transformation3 &other_odometry)
{
  Transformation3 motion = m_state.reference_odom.inverse() * odometry;
  Transformation3 other_motion = m_state.reference_other_odom.inverse() * other_odometry;
  m_motion_model.update_motion(motion, other_motion);
  //     m_gps.update_motion(motion);
  //     m_laser.update_motion(motion);
  //     m_vision.update_motion(motion);

  // Node id update handling
  Vector6 motion6d = motion.toVector();
  Vector6 other_motion6d = motion.toVector();
}


/**
 * \brief Updates the filter using the laser sensor model.
 *
 * Updates the state of the particle filter using the sensor model for the
 * laser scanner.
 *
 * \param measurement latest scan measurement to be used in the weight
 *        computation
 * \return true if an update was performed, false otherwise
 */
bool Localization::update_laser(const std::vector<Vector3f>& endpoints, const Transformation3& odomPose, const Transformation3& otherOdomPose, const double& timestamp)
{

  // Perform global localization, even while standing still if the system
  // never has been localized so far.
  if(m_active_localization) {
    for(ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
      Vector6 vec;
      vec.fill(0.0);
      vec[0] = (drand48() - 0.5) * 0.5;
      vec[1] = (drand48() - 0.5) * 0.5;
      vec[5] = (drand48() - 0.5) * 0.5;
      Transformation3 jitter = Transformation3::fromVector(vec);

      it->pose *= jitter;
    }
    m_laser.compute_weights(m_particles, endpoints);
    update_filter(m_laser, odomPose, otherOdomPose, timestamp);
    if(m_active_localization_laser_count < 500){
      m_active_localization = !m_laser.localized();
    } else {
      set_localized(m_laser.localized());
    }
    m_active_localization_laser_count++;
    //         cerr << (int) m_active_localization_laser_count << " " << PVAR(m_laser.localized()) << " " << PVAR(m_active_localization) << endl;
    return m_laser.localized();
  }

  if(!m_laser.perform_update(m_motion_model.linear_motion(), m_motion_model.angular_motion())) {
    // 	cerr << "\tUpdate not performed" << endl;
    return false;
  }

  propagate_particles();
  m_laser.compute_weights(m_particles, endpoints);
  update_filter(m_laser, odomPose, otherOdomPose, timestamp);
  set_localized(m_laser.localized());
  reset_motion();
    
  if(!isLocalized()) {
    // 	cerr << "\tUpdate performed, robot not localized" << endl;
    return false;
  }

  return true;
}

/**
 * \brief Returns the pose of the best particle.
 *
 * \return 3D pose of the best particle
 */
Transformation3 Localization::best_pose() const
{
  double best_weight = -1e9;
  Transformation3 best_pose;
  for(ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    if(best_weight < it->weight) {
      best_weight = it->weight;
      best_pose = it->pose;
    }
  }
  return best_pose;
}

/**
 * \brief Returns information about the convergence state of the system.
 *
 * \param mean storage for the mean
 * \param cov storage for the covariance matrix
 * \param bounded storage for the bounded information
 * \return true if the system is localized according to the laser sensor, false otherwise
 */
bool Localization::has_converged(
    Transformation3            &mean,
    Matrix6                    &cov,
    bool                       &bounded
                                 ) const
{
  Vector6 motion_vec = m_motion_model.cummulative_motion().toVector();
  Transformation3 motion(
      Vector3(motion_vec[0], motion_vec[1], 0.0),
      Quaternion(0.0, 0.0, motion_vec[5])
                         );
  //mean    = m_mean;
  Vector6 tmp;
  tmp[0] = m_mean.toVector()[0];
  tmp[1] = m_mean.toVector()[1];
  tmp[2] = 0.0;
  tmp[3] = 0.0;
  tmp[4] = 0.0;
  tmp[5] = m_mean.toVector()[5] + motion_vec[5];

  mean    = mean.fromVector(tmp);
  cov     = m_covariance;
  bounded = m_bounded;
  bool laser_localized = m_laser.localized();
  return laser_localized;
}


/**
 * \brief return a pointer to the computed distance map
 *
 */
AISNavigation::FloatMap*  Localization::get_d_map(){
  return m_laser.distance_map();
}

/**
 * \brief Sets the number of particles used in the filter.
 *
 * \param num number of particles to use
 */
void Localization::particles(int num)
{
  m_particles.resize(num);
}

/**
 * \brief Returns the current number of particles used.
 *
 * \return number of currently used particles
 */
int Localization::particles() const
{
  return m_particles.size();
}

/**
 * \brief Returns the current particle vector.
 *
 * \return current vector of particles
 */
void Localization::getParticles(ParticleVector*& particles, bool old)
{
  if(old) {
    particles = &m_particles_old;
  } else {
    particles = &m_particles;
  }
}

void Localization::set_map(AISNavigation::FloatMap *map)
{
  m_map = map;
  m_laser.set_map(map);
}

void Localization::set_initialized(bool state)
{
  m_state.initialized = state;
}

void Localization::set_localized(bool state)
{
  m_state.localized = state;
}

/**
 * \brief Changes the particle poses according to the motion model.
 */
void Localization::propagate_particles()
{
  m_motion_model.prepare_sampling(m_motion_model.cummulative_motion(), 0.0);
  //cerr << "CUMMOTION: " << (m_motion_model.cummulative_motion()).toVector() << endl;
  for(ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    Transformation3 pose_change = m_motion_model.sample_pose_change(m_motion_model.cummulative_motion());
    it->pose =  it->pose * pose_change;
  }
  //     reset_motion();
}

/**
 * \brief Updates the internal state of the filter.
 */
void Localization::update_state(const Transformation3& odomPose, const Transformation3& otherOdomPose, const double& timestamp)
{
  compute_mean();
  compute_covariance();

  determine_convergence();

  m_state.reference_pose = m_mean;
  m_state.reference_odom = odomPose;
  m_state.reference_other_odom = otherOdomPose;
  m_state.reference_timestamp = timestamp;
}

/**
 * \brief Computes the weighted mean of the particle cloud.
 *
 */
void Localization::compute_mean()
{
  double total_weight = 0.0;
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  Vector3 sum_pos(0.0, 0.0, 0.0);
  for(ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    double weight = exp(it->weight);
    Vector6 pose = it->pose.toVector();
    sum_pos[0] += pose[0] * weight;
    sum_pos[1] += pose[1] * weight;
    sum_pos[2] += pose[2] * weight;
    sum_cos += cos(pose[5]) * weight;
    sum_sin += sin(pose[5]) * weight;
    total_weight += weight;
  }

  // If all particles have zero weight set the mean to zero as well.
  // This prevents division by zero and subsequent bad things.
  sum_pos[0] = total_weight > 1e-10 ? sum_pos[0] / total_weight : 0.0;
  sum_pos[1] = total_weight > 1e-10 ? sum_pos[1] / total_weight : 0.0;
  sum_pos[2] = total_weight > 1e-10 ? sum_pos[2] / total_weight : 0.0;
  double angle = total_weight > 1e-10 ? atan2(sum_sin, sum_cos) : 0.0;
  m_mean = Transformation3(sum_pos, Quaternion(0.0, 0.0, angle));
}

/**
 * \brief Computes the covariance from the weighted particles.
 *
 */void Localization::compute_covariance()
{
  double total_weight = 0.0;
  m_covariance.fill(0.0);

  Vector6 mean = m_mean.toVector();
  for(ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    double weight = exp(it->weight);
    Vector6 pose = it->pose.toVector();
    double dx = pose[0] - mean[0];
    double dy = pose[1] - mean[1];
    double dz = pose[2] - mean[2];
    double dyaw = pose[5] - mean[5];
    dyaw = atan2(sin(dyaw), cos(dyaw));

    m_covariance[0][0] += weight * dx * dx;
    m_covariance[0][1] += weight * dx * dy;
    m_covariance[0][2] += weight * dx * dz;
    m_covariance[0][5] += weight * dx * dyaw;

    m_covariance[1][0] += weight * dy * dx;
    m_covariance[1][1] += weight * dy * dy;
    m_covariance[1][2] += weight * dy * dz;
    m_covariance[1][5] += weight * dy * dyaw;

    m_covariance[2][0] += weight * dz * dx;
    m_covariance[2][1] += weight * dz * dy;
    m_covariance[2][2] += weight * dz * dz;
    m_covariance[2][5] += weight * dz * dyaw;

    m_covariance[5][0] += weight * dyaw * dx;
    m_covariance[5][1] += weight * dyaw * dy;
    m_covariance[5][2] += weight * dyaw * dz;
    m_covariance[5][5] += weight * dyaw * dyaw;

    total_weight += weight;
  }

  m_covariance = m_covariance * (1.0 / total_weight);
}

/**
 * \brief Determines if the filter has converged.
 *
 * \param type sensor type for which the convergence is to be determined
 */
void Localization::determine_convergence()
{
  // We only use the x,y coordinates as z is always 0 (at least for now) and
  // using the z coordinate results in small values with no particular meaning
  Matrix2 metric;
  metric[0][0] = m_covariance[0][0];
  metric[0][1] = m_covariance[0][1];
  metric[1][0] = m_covariance[1][0];
  metric[1][1] = m_covariance[1][1];
    
  //     cerr << "Metric det: " << metric.det() << ", Metric: \n" << metric << endl;

  double metric_area = sqrt(metric.det());
  double angular_area = sqrt(m_covariance[5][5]);

  m_laser.filter_state(metric_area, angular_area, m_motion_model.linear_motion(), m_active_localization);
}

/**
 * \brief Performs the particle filter resampling step.
 */
void Localization::perform_resampling()
{
  // Convert weights from log to normal space
  // TODO: (Lionel) convert weight back to logspace afterwards?
  double log_max = *max_element(m_particles.begin(), m_particles.end());
  for(ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    it->weight = exp(it->weight - log_max);
  }

  // Resmpling with "standard" PF
  //    vector<size_t> indices;
  //    resample(indices, m_particles);
  //    ParticleVector resampled = m_particles;
  //    repeatIndexes(resampled, indices, m_particles);

  // KLD based adaptive resampling
  ParticleVector resampled;
  resampled.reserve(m_particles.size());
  
  resampleKLD(resampled, m_particles, m_localizer_params.particles);
 
  // std::cerr << "Particle size change from " << m_particles.size() << " to " << resampled.size() << std::endl;

  m_particles = resampled;

  for(ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    it->weight = log(it->weight);
  }
}

/**
 * \brief Resets motion information of all sensors.
 */
void Localization::reset_motion()
{
  m_motion_model.reset();
}

/**
 * \brief Updates the filter and resets the sensors used.
 *
 * \param sensor sensor model to reset the motion
 */
void Localization::update_filter(BasicSensor &sensor,  const Transformation3& odomPose, const Transformation3& otherOdomPose, const double& timestamp)
{
  m_particles_old = m_particles;

  //double var = particle_weight_variance();
  double neff = particle_weight_neff();
  if(neff < (m_localizer_params.resample_variance * m_particles.size())) {
    perform_resampling();
  }
  update_state(odomPose, otherOdomPose, timestamp);
  //     m_motion_model.reset();
}

/**
 * \brief Returns the variance of the particle weights.
 *
 * \return variance of the particle weights
 */
double Localization::particle_weight_neff() const
{
  double neff = 0.0;
  double log_max = *max_element(m_particles.begin(), m_particles.end());
  for(ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    neff += pow(exp(it->weight - log_max), 2);
  }

  return neff;
}


/**
 * \brief Returns the variance of the particle weights.
 *
 * \return variance of the particle weights
 */
double Localization::particle_weight_variance() const
{
  double mean = 0.0;
  double log_max = *max_element(m_particles.begin(), m_particles.end());
  for(ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    mean += exp(it->weight - log_max);
  }
  mean /= m_particles.size();

  double var = 0.0;
  for(ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); ++it) {
    var += pow(exp(it->weight - log_max) - mean, 2);
  }
  var /= (m_particles.size() - 1);
  return var;
}

/**
 * \brief Samples a new free space pose from the graph.
 *
 * \return 3D pose of in the map
 */
Transformation3 Localization::random_new_pose() 
{
  // Get random vertex from the graph
  boost::uniform_real<> distribution_X(0.0, m_map->size().x()*m_map->resolution());
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> > gen_X(m_rand_engine, distribution_X);
  boost::uniform_real<> distribution_Y(0.0, m_map->size().y()*m_map->resolution());
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> > gen_Y(m_rand_engine, distribution_Y);
  boost::uniform_real<> distribution_T(-M_PI, M_PI);
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> > gen_T(m_rand_engine, distribution_T);
  Transformation3 pose(Vector3(gen_X() + m_map->offset().x(), gen_Y() + m_map->offset().y()), Quaternion(0.,0.,gen_T()));
  return pose;
}

/**
 * \brief Generates a random 6D pose with the given parameters.
 *
 * Generates a 6D pose where by generating a random offset to the mean where
 * only the x, y, yaw values are varied.
 *
 * \param mean meann of the random distributino
 * \param sigma_trans sigma for the translational component
 * \param sigma_rot sigma for the rotational component
 */
Transformation3 Localization::random_6d_pose(
    const Transformation3          &mean,
    double                          sigma_trans,
    double                          sigma_rot
                                             ) 
{
  boost::normal_distribution<> distribution_trans(0.0, sigma_trans);
  boost::normal_distribution<> distribution_rot(0.0, sigma_rot);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > gen_trans(m_rand_engine, distribution_trans);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > gen_rot(m_rand_engine, distribution_rot);
  Transformation3 offset(
      Vector3(gen_trans(), gen_trans(), 0.0),
      Quaternion(0.0, 0.0, gen_rot())
                         );

  return mean * offset;
}

void Localization::reload_parameters()
{
  particles(m_localizer_params.particles);
  m_particles_old = ParticleVector(m_particles.size());
  m_motion_model.set_params(&m_motion_model_params);
  m_laser = LaserSensor(m_localizer_params.max_distance, m_localizer_params.min_occupancy, m_localizer_params.mark_unknown);
  m_laser.set_params(&m_laser_sensor_params);
}

// 
// Localization.cpp ends here
