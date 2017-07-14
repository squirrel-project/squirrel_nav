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

### Parameters
Parameters of `squirrel_navigation::LocalPlanner`:
- `~/verbose` set verbosity.
- `~/odom_topic` the odometry topic.
- `~/goal_{lin/ang}_tolerance` distance (xy-coordinate) from goal to be
  considered reached.
- `~/max_safe_{lin/ang}_velocity` maximum linear velocity to be actuated.
- `~/max_safe_{lin/ang}_displacement` maximum displacement from the
  reference position (pid controller) to ask for replanning.
- `~/safety_observers` (`SafetyScanObserver`, `ArmSkinObserver`) robot
  state observers (**not stable yet**).
- `~/MotionPlanner/max_<linear/angular>_velocity` maximum velocities used
  during the velocity planning phase.
- `~/MotionPlanner/{linear/angular}_smoother` smoothing parameter for the path
- `~/MotionPlanner/time_scaler` global velocity rescaler for the
  velocity planning phase.
- `~/MotionPlanner/waypoints_heading_lookahead` Used when
  `~/move_base/planner_frequency` in not zero. Waypoints lookahead in
  for the new planned path.
- `~/MotionPlanner/lookahead` temporal lookahead for the reference
  pose (pid controlller).
- `~/ControllerPID/k{P/I/D}_{ang/lin}` Controller gains for the linear
  and rotational velocity.

Parameters of `squirrel_navigation::FootprintPlanner` (RRT* planner):
- `~/FootprintPlanner/verbose` set verbosity.
- `~/FootprintPlanner/footprint_topic` the footprint of the robot.
- `~/FootprintPlanner/collision_check_resolution` check OMPL doc.
- `~/FootprintPlanner/max_planning_time` see OMPL doc.
- `~/FootprintPlanner/max_simplification_time` see OMPL doc.
- `~/FootprintPlanner/map_resolution` see OMPL doc.
- `~/FootprintPlanner/range` see OMPL doc.

Paramters of `squirrel_navigation::GlobalPlanner`:
- `~/verbose` set verbosity.
- `~/plan_with_footprint` whether to plan with dijkstra or RRT*.
- `~/plan_with_constant_heading` the resulting path has constant
  yaw. Usable only if `plan_with_footprint` is not enabled.
- `~/heading` the constant heading to use if
  `plan_with_constant_heading` is enabled.
- `~/Dijstra/*` parameters of `nav_core::NavFnROS`.
- `~/RRTstar/*` parameters of
  `squirrel_navigation::FootprintPlanner`.
  

## SQUIRREL Costmap layers

Contains `squirrel_navigation::NavigationLayer` which merges obstacles
detected with depth camera and the safety laser rangefinder.

### Parameters
- `~/use_kinect` whether to use or not the kinect.
- `~/use_laser` whether to use or not the laser scan.
- `~/LaserLayer/*` parameters of `costmap_2d::ObstacleLayer`.
- `~/KinectLayer/*` parameters of `costmap_2d::VoxelLayer`.
- `~/StaticLayer/*` parameters of `costmap_2d::StaticLayer`.

### Services.
- `~/clearCostmapRegion`
  (`squirrel_navigation_msgs::ClearCostmapRegion`) clears the costmap
  with the region specified by a polygon.
- `~/getObstaclesMap` (`squirrel_navigation_msgs::GetObstacleMap`)
  returns the position of all the obstacles in the map as well as an
  occupied/free boolean for every pixel in the map.
- `/getPathClearance` (`squirrel_navigation_msgs::GetPathClereance`)
  returns the clearance of a path as well as the proximity map of
  every waypoint of the path.
 
