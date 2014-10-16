navigation
==========

Repository for navigation related SQUIRREL packages.

##Creating a 2D Gridmap
```bash
roslaunch robotino_navigation gmapping.launch
```
We prepaired a configuration file (for rviz-hydro), that can be found in the git repository at
*alufr_navigation\config\rviz\hydro_config.rviz*.
You can load it in rviz via *Cltr+O*. You should see a small part of a map.
Move the robotino with the joystick to complete the map.
If done, save the map with the following command:
```bash
rosrun map_server map_saver -f [mymap]
```
where *[mymap]* should be replaced by the correct path and filename.

##Starting Laser Based Navigation in the 2D Map
###Launch Navigation
```bash
roslaunch robotino_navigation navigation.launch map_file:=[map.yaml]
```
*[map.yaml]* has to be replaced by the recorded mapping file.
You have to use the full path to the mapping file, even if the yaml file is in the same directory.

###Start Rviz
To set a initial pose estimation, start rviz on your desktop.
```bash
rosrun rviz rviz
```
There is rviz configuration files in our git repository *alufr_navigation\config\rviz\hydro_config.rviz*,
that can be loaded with *Ctrl+O*.

###Set Initial Pose and Move with Rviz
Set at first a pose estimate with the button *2D Pose Estimate* and move the robot arround the map to localize the
robot. After that, you can send a navigation goal via the rviz button *2D Nav Goal*.The robot should now move to the goal.

##Creating a 3D Octomap Localizing in a 2D Map
As as first step, start the navigation as described above.

TODO: finish


##TODO: Using RGBD-SLAM to create an 3D OCTOMAP




