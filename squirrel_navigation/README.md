squirrel_navigation
===================

Autonomous navigation for SQUIRREL

## Nodes and plugins

The package provides the following nodes and plugins for [move_base](http://wiki.ros.org/move_base)

- `point_cloud_filter_node`: Point Cloud downsampler.
- `planner_udpate_handle_node`: Toggle the lattice planner
- `squirrel_navigation::GlobalPlanner`: Path planner.
- `squirrel_navigation::LocalPlanner`: Local planner and controller
- `squirrel_navigation::DownprojectionLayer`: Layer for obstacles
- `squirrel_navigation::DownprojectionMultilayer`: Mutlilayer for obstacles
- `squirrel_navigation::MapLayer`: Layer for the map

### Point Cloud Downsampler

#### Subscribed topics
- `/cloud_in` (*sensor_msgs/PointCloud2*) input point cloud

#### Published topics
- `/cloud_out` (*sensor_msgs/PointCloud2*) the downsampled point cloud;

#### Parameters
- `~/pointcloud_in`: topic of the input point cloud.
- `~/pointcloud_out`: topic of the output point cloud.
- `~/pointcloud_size`: size of the output point cloud.
- `~/nan_free`: whether NaN values has been already filtered out.


### Planner Update Handler

#### Services
- `/updatePlanners`(*squirrel_navigation_msgs/PlannerUpdate*) toggle the lattice planner

#### Published Topics
- `/plan_with_footprint` (*std_msgs/Bool*) plan using footprint.

### Global Planner

Uses [navfn](http://wiki.ros.org/navfn) and
[sbpl_lattice_planner](http://wiki.ros.org/sbpl).

#### Subscribed topic
- `/odom` (*nav_msgs/Odomerty*) Odometry.
- `/plan_with_footprint` (*std_msgs/Bool*) toggle the lattice planner.

#### Published topics
- `~/plan` (*nav_msgs/Path*) the path
- `~/poses` (*geometry_msgs/PoseArray*) the poses for the path compute by the lattice planner (only if `verbose` is on) 
- `~/stats` (*squirrel_navigation_msgs/GlobalPlannerStats*) output of the lattice planner
  
#### Parameters
- `~/verbose`: verbosity.
- `~/heading_lookhahead`: Time lookahead for replanning along the path.
- `~/dijkstra/<params>`: see paramters for [navfn](http://wiki.ros.org/navfn).
- `~/lattice/<param>`: see parameters for [sbpl_lattice_planner](http://wiki.ros.org/sbpl_lattice_planner).

### Local Planner

Uses [base_local_planner](http://wiki.ros.org/base_local_planner).

#### Subscribed topic
- `/plan_with_footprint` (*std_msgs/Bool*) toggle the dwa controller.

#### Published topics
- `~/cmd_vel` (*geometry_msgs/Twist*) the control command.
- `~/ref_pose` (*visualization_msgs/Marker*) the tracked reference pose.
  
#### Parameters
- `~/verbose`: verbosity.
- `~/odom_topic`: The odometry topic.
- `~/xy_goal_tolerance`: Goal tolearance in 


<!-- - `~/trajectory_planner/<params>`: see paramters for [base_local_planner](http://wiki.ros.org/base_local_planner). -->
<!-- - `~/trajectory_tracker/max_linear_vel`: maximum forward speed. -->
<!-- - `~/trajectory_tracker/min_linear_vel`: minimum forward speed. -->
<!-- - `~/trajectory_tracker/max_rotation_vel`: maximum rotational speed. -->
<!-- - `~/trajectory_tracker/min_rotation_vel`: minimum rotational speed. -->
<!-- - `~/trajectory_tracker/max_in_place_rotation_vel`: maximum rotational speed on the spot. -->
<!-- - `~/trajectory_tracker/min_in_place_rotation_vel`: minimum rotational speed on the spot. -->
<!-- - `~/trajectory_tracker/yaw_goal_tolerance`: angular tolerance to reach the goal. -->
<!-- - `~/trajectory_tracker/xy_goal_tolerance`: linear tolerance to reach the goal. -->
<!-- -  `~/trajectory_tracker/heading_lookahead`: lookahead distance. -->
<!-- -  `~/trajectory_tracker/num_window_points`number of lookahaed points: . -->
<!-- -  `~/trajectory_tracker/holonomic_robot`: holonomic. -->
