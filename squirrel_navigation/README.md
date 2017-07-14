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
 
