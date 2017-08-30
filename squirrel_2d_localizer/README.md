squirrel_2d_localizer
=====================

Standard 2d Monte Carlo Localization for SQUIRREL

## Nodes

The package provide a single node

- `squirrel_2d_localizer_node`: Run MCL algorithm.

It requires the service `/static_map` published by `map_server` node
to retrieve the map.

## `squirrel_2d_localizer_node`

### Parameters
- `~/map_frame` (defualt `/map`): the world fixed frame
- `~/odom_frame` (default `/odom`): the odometry frame
- `~/robot_frame` (default `/base_link`): the robot frame
- `~/init_pose_{x,y,a}` (default `{0.0, 0.0, 0.0}`): initial guess
  pose.
- `~/localizer/num_particles` (default: `750`): number of particles for
  the particle filter.
- `~/localizer/min_lin_update` (default `0.5`): minimum (cumulative) linear
  distance before performing a filter update.
- `~/localizer/min_ang_update` (default `0.5`): minimum (cumulative) angular
  distance before performing a filter update.
- `~/localizer/init_stddev_{x,y,a}` (default `{0.5, 0.5, 0.5}`): initial
  standard deviations of particles (Gaussian distributed).
- `~/motion_model/noise_{xx, xy, xa, yy, ya, aa}` (default `{1.0, 0.0, 0.0, 1.0,
  0.0, 1.0}`), noise components of the odometry model.
- `~/motion_model/noise_magnitude` (default `1.0`): rescaling factor for the noise
  parameters.
- `~/latent_model_likelihood_field/saturation_distance` (default `0.5`): saturation distance of
  the Gaussian kernels.
- `~/latent_model_likelihood_field/observation_sigma` (default `0.5`): variance
  parameter of the gaussian kernels.
- `~/laser_model/beam_min_distance` (default `0.1`) downsampling
  factor for the laser readings.
- `~/publish_extra_tf` relay the transformation between
  `~/map_frame_id` to `~/odom_frame_id` to extra frames.
- `~/extra_parent_frame_id` frame ID of the extra transformation.
- `~/extra_child_frame_id` frame ID of the extra transformation.

### Advertised Services
Uses messages provided by [squirrel_2d_localization_msgs](https://github.com/squirrel-project/squirrel_common/tree/indigo_dev/squirrel_3d_localizer_msgs)
- `~/globalLocalization`
  (`squirrel_2d_localizer_msgs::GlobalLocalization`) distribute
  particles all over the free space of the map.

### Advertised Topics
- `/tf`: transform from `map_frame` to `odom_frame`.
- `~/pose` (*geometry_msgs/PoseWithCovarianceStamped*): the robot pose
  in `map_frame`.
- `~/particles` (*geometry_msgs/PoseArray): the particle set in `map_frame`.

### Subscriptions
- `/scan`, (*sensor_msgs/Scan*) the laser scan.
- `/tf`, transformation from the `robot_frame` and the `odom_frame` as well
  as from `robot_frame` to the sensor link.
- `/initialpose` (*geometry_msgs/PoseWithCovarianceStamped*) initial
  guess.
