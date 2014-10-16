navigation
==========

Repository for navigation related SQUIRREL packages.

##Creating a 2D Gridmap
```bash

##Starting Laser Based Navigation in the 2D Map

```bash
roslaunch robotino_navigation navigation.launch map_file:=[/full/path/to/mapping.yaml]
```
*[/full/path/to/mapping.yaml]* has to be replaced by the recorded
mapping file (see \ref{mapping}). You have to use the full path to the mapping file,
even if the yaml file is in the same directory.

To set a initial pose estimation, start rviz on your desktop.
```bash
rosrun rviz rviz
```
There is rviz configuration files in our git repository *alufr_navigation/config*,
that can be loaded with *Ctrl+O*.
Set at first a pose estimate with the button *2D Pose Estimate*
and move the robot arround the map to localize the
robot. After that, you can send a navigation goal via the rviz button *2D Nav Goal*.
The robot should now move to the goal.



##Creating a 3D Octomap Localizing in a 2D Map


##Using RGBD-SLAM to create an 3D OCTOMAP



