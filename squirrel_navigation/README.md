squirrel_navigation
===================

Autonomous navigation for SQUIRREL

## Libraries, nodes and plugins

- `squirrel_navigation_utils`: Utility library used in the package.
- `squirrel_navigation_safety`: Safety tools for navigation.
- `squirrel_navigation_planners`: Local and global planners.
- `squirrel_navigation_costmap_layer`: Costmap layer used for
  navigation.

## SQUIRREL Planners

Contains `squirrel_navigation::GlobalPlanner`,
`squirrel_navigation::FootprintPlanner` and
`squirrel_navigation::LocalPlanner` used for 2D navigation.

### Local Planner (`squirrel_navigation::LocalPlanner`)

#### Parameters

- `~/LocalPlanner/verbose` set verbosity.
- `~/LocalPlanner/visualize_topics` publish the visualization topics (defalut **true**)
- `~/LocalPlanner/odom_topic` the odometry topic.
- `~/LocalPlanner/goal_{lin, ang}_tolerance` distance from goal to be considered reached.
- `~/LocalPlanner/max_safe_{lin, ang}_velocity` maximum linear velocity to be
  actuated.
- `~/LocalPlanner/max_safe_{lin, ang}_displacement` maximum displacement from the
  reference position (pid controller) to ask for replanning.
- `~/LocalPlanner/collision_based_replanning` whether to trigger replanning based
  on collisions of the forward trajectory instead of time based.
- `~/LocalPlanner/replanning_{lin, ang}_lookahead` lookahead for the replanning
  trigger.
- `~/LocalPlanner/replanning_path_length_ratio` Ratio of length between the old
  path and the candidate replanned. If the new path is shorter accept.
- `~/LocalPlanner/safety_observers` (`SafetyScanObserver`, `ArmSkinObserver`) robot
  state observers (**not stable yet**).
- `~/LocalPlanner/MotionPlanner/max_{linear, angular}_velocity` maximum velocities
  used during the velocity planning phase.
- `~/LocalPlanner/MotionPlanner/{linear, angular}_smoother` smoothing parameter for the path
- `~/LocalPlanner/MotionPlanner/time_scaler` global velocity rescaler for the
  velocity planning phase.
- `~/LocalPlanner/MotionPlanner/waypoints_heading_lookahead` Used when
  `~/LocalPlanner/move_base/planner_frequency` in not zero. Waypoints lookahead
  for the new planned path.
- `~/LocalPlanner/MotionPlanner/lookahead` temporal lookahead for the reference
  pose (controller).
- `~/LocalPlanner/ControllerPID/k{P, I, D}_{ang, lin}` Controller gains for the linear
  and rotational velocity.
- `~/LocalPlanner/ControllerPID/visualize_topics` Publish the
  visualization messages (defualt **true**).

#### Advertised Topics
- `~/LocalPlanner/cmd_navigation` (`visualization_msgs::MarkerArray`) the control
  output by the local planner.
- `~/LocalPlanner/reference_pose` (`visualization_msgs::Marker`) the reference pose
  currently tracked.
- `~/LocalPlanner/trajectory` (`geometry_msgs::PoseArray`) the planned trajectory.
- `~/LocalPlanner/footprints` (`visualization_msgs::MarkerArray`) the
  robot footprint on the planned trajectory.
- `~/LocalPlanner/ControllerPID/cmd_raw` (`visualization_msgs::MarkerArray`) the raw
  control output by the controller.

#### Subscriptions
- `/odom` the odometry topic (reconfigurable).

#### Advertised Services
Uses messages provided by [squirrel_navigation_msgs](https://github.com/squirrel-project/squirrel_common/tree/indigo_dev/squirrel_navigation_msgs).
- `~/LocalPlanner/brakeRobot` (`squirrel_nav_msgs::BrakeRobot`) stop the robot for
  a certain time.
- `~/LocalPlanner/unbrakeRobot` (`std_srvs::Empty`) release the brake.

### Footprint Planner (`squirrel_navigation::FootprintPlanner`)

#### Parameters 

This planner is wrapper around [SBPL ARA* planner](http://www.sbpl.net/):
- `~/FootprintPlanner/verbose` set verbosity.
- `~/FootprintPlanner/visualize_topics` publish the visualization topics (default **true**)
- `~/FootprintPlanner/footprint_topic` the footprint of the robot.
- `~/FootprintPlanner/forward_search` see SBPL documentation.
- `~/FootprintPlanner/max_planning_time` Maximum time assigned for
  searching a collision free path.
- `~/FootprintPlanner/iintial_epsilon` see SBPL documentation.
- `~/FootprintPlanner/motion_primitive_url` filename containing the
  motion primitives. See SBPL documentation.

#### Advertised Topics
- `~/plan` (`nav_msgs::Path`) the computed plan.
- `~/waypoints` (`geometry_msgs::PoseArray`) the computed waypoints.
- `~/footprints` (`geometry_msgs::MarkerArray`) the sequence of
  footprints of the robot on the path's waypoints.


#### Subscriptions
- `/footprint_observer/footprint` the footprint of the robot
  (reconfigurable).
  
### Global Planner (`squirrel_navigation::GlobalPlanner`)

#### Parameters 

Parameters of `squirrel_navigation::GlobalPlanner`:
- `~/GlobalPlanner/verbose` set verbosity.
- `~/GlobalPlanner/visualize_topics` publish the visualization topics (default **true**)
- `~/GlobalPlanner/plan_with_footprint` whether to plan with dijkstra or RRT*.
- `~/GlobalPlanner/plan_with_constant_heading` the resulting path has constant
  yaw. Usable only if `plan_with_footprint` is not enabled.
- `~/GlobalPlanner/heading` the constant heading to use if
  `plan_with_constant_heading` is enabled.
- `~/GlobalPlanner/Dijkstra/*` parameters of [`nav_core::NavFnROS`](http://wiki.ros.org/navfn).
- `~/GlobalPlanner/ARAstar/*` parameters of `squirrel_navigation::FootprintPlanner`.
  
#### Advertised Topics  
- `~/GlobalPlanner/Dijkstra/*` topics advertised by `nav_core::NavFnROS`.
- `~/GlobalPlanner/ARAstar/*` topics advertised by `squirrel_navigation::FootprintPlanner`.
- `~/GlobalPlanner/plan` (`nav_msgs::Path`) the path computed by the planner.
- `~/GlobalPlanner/waypoints` (`geometry_msgs::PoseArray`) the waypoints computed by the planner.
- `~/footprints` (`geometry_msgs::MarkerArray`) the sequence of
  footprints of the robot on the waypoints.

## SQUIRREL Costmap layers

Contains `squirrel_navigation::NavigationLayer` which merges obstacles
detected with depth cameras and the safety laser rangefinders.

### Parameters
- `~/use_kinect` whether to use or not the kinect.
- `~/use_laser` whether to use or not the laser scan.
- `~/LaserLayer/*` parameters of [`costmap_2d::ObstacleLayer`](http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1ObstacleLayer.html).
- `~/DepthCameraLayer/*` parameters of [`costmap_2d::VoxelLayer`](http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1VoxelLayer.html).
- `~/StaticLayer/*` parameters of [`costmap_2d::StaticLayer`](http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1StaticLayer.html).

### Advertised Services
Uses messages provided by [squirrel_navigation_msgs](https://github.com/squirrel-project/squirrel_common/tree/indigo_dev/squirrel_navigation_msgs).
- `~/clearCostmapRegion`
  (`squirrel_navigation_msgs::ClearCostmapRegion`) clears the costmap
  with the region specified by a polygon.
- `~/getObstaclesMap` (`squirrel_navigation_msgs::GetObstacleMap`)
  returns the position of all the obstacles in the map as well as an
  occupied/free boolean for every pixel in the map.
- `~/getPathClearance` (`squirrel_navigation_msgs::GetPathClereance`)
  returns the clearance of a path as well as the proximity map of
  every waypoint of the path.


## Know Issues
On shutdown, `ClassLoader` throws an error. It should only happens on
exit and therefore not influence the navigation stack. 
