squirrel_localizer
==================

Monte Carlo Localization for SQUIRREL

## Nodes

The package provides two nodes
1. `squirrel_localizer_localization_node`: Perform AMCL.
2. `squirrel_localizer_tf_pose_node`: Broadcast the tranform `/map` to `/odom`

### Localization node

#### Subscribed topics
- `/front_scan` (*sensor_msgs/LaserScan*) the front scans.
- `/rear_scan` (*sensor_msgs/LaserScan*) the rear scans (if enabled).
- `/tf` tranformation from `/odom` to `/base_link` and form `/odom` to `/odom_scan` (if enabled).
- `/map` (*nav_msgs/OccupancyGrid*) a map encoded as an occupancy grid. 
- `/initialpose` (*geometry_msgs/PoseWithCovarianceStamped*) initial guessed pose.
  
#### Published topics
- `/squirrel_localizer_pose` (*geometry_msgs/PoseWithCovarianceStamped*), the current pose
- `/squirrel_localizer_odom_pose` (*geometry_msgs/PoseWithCovarianceStamped*), the incremental pose
- `/particlecloud` the particles.

#### Parameters

- `~/verbose`: verbosity.
- `~/number_of_particles`: maximum number of particles used for AMCL.
- `~/maxLocalizationRange`: maximum range used for the rangefinder sensors.
- `~/maxRange`: maximum range of the laser rangefinder.
- `~/motion_param_<id>`: parameters for the proposal distrubution.
- `~/dMapThreshold`: farthest distance from obstacles to compute the distance transform.
- `~/observationSigma`: noise in the observation.
- `~/use_second_laser`: use rear laser rangefinder.
- `~/use_laser_odom`: use second odometry source from scan matcher
- `~/<name>_frame_ID`: reference frames.
- `~/<name>_topic`: scan topics.
- `~/use_last_pose`: on start up, use last pose as initial guess.
- `~/use_scan_to_map_matching`: use scan to map matching to correct localization
- `~/sm_point2line_outliers_maxPerc`: maximum percentage of outliers for scan to map matcher  
- `~/sm_fitler_gain`: filter gain for filtering the pose when using scan to map matcher
  
### Pose Broadcaster

#### Subscribed topic
- `/squirrel_localizer_pose` (*geometry_msgs/PoseWithCovarianceStamped*), the current pose
  
#### Published topic
- `/tf` the transformation form `/map` to `/odom`.

#### Parameters
- `~/pose_topic`: the (incremental) pose topic.
- `~/from_id`: fixed frame.
- `~/to_id`: the odometry frame.

## Launch files

There are two launch files avalilable
1. `squirrel_localizer.launch`: run Monte Carlo Localization a single
odometry source.
2. `squirrel_localizer_odom_scan.launch`: run Monte Carlo Localization
using odometry from the encoders and from
[laser_scan_matcher](https://github.com/federico-b/squirrel_nav/tree/indigo_dev/laser_scan_matcher).
