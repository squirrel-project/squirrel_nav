navigation
==========

Repository for navigation related SQUIRREL packages.

## Requirement

Install robotino driver's and robotino safety node from the common
repository

## Creating a 2D Gridmap

Run the mapping routine by executing

```bash
roslaunch squirrel_navigation gmapping.launch 
``` 

The map created can be then visualised in RViz. The robot should be
teleoperated to explore the area. Once the map of the scene is
correctly computed, it can be stored running

```bash
roslaunch squirrel_navigation gmapping.launch
```
An rviz configuration file, can be found in the git repository at
*alufr_navigation\config\rviz\hydro_config.rviz*.
You can load it in rviz via *Cltr+O*. You should see a small part of a map.
Move the robotino with the joystick to complete the map.
If done, save the map with the following command:
```bash
rosrun map_server map_saver -f [mymap]
```
where *[mymap]* should be replaced by the correct path and filename.
`mymap.pgm` and `mymap.yaml` are created in the folder where the
above command has been executed.

## Starting Laser Based Navigation in the 2D Map

```bash
roslaunch squirrel_navigation navigation.launch map_file:=[mymap.yaml]
```
*[mymap.yaml]* has to be replaced by the recorded mapping file.
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

### Localising in a 2D Gridmap

Set at first a pose estimate with the button *2D Pose Estimate* and
move the robot around the map to localise the robot.

### Performing a navigation task

After that, you can send a navigation goal via the rviz button *2D Nav Goal*.
The robot should now move to the goal.


## Creating a 3D Octomap 

To generate a 3D map of scene the pose of the robot must be known. For this, perform the
localisation procedure as described in the section above. Once the robot is localised, 
launch *octomap_server*:

```bash
roslaunch squirrel_navigation octomap_server.launch
```

Let the robot navigate in the environment. Finally store the 3D Octomap executing

```bash
rosrun octomap_server octomap_saver -f [octomap_name.ot]
```

The 3D Octomap should now have been stored in *octomap_name.ot*.

### Visualising the 3D Octomap

To visualise the octomap, `octovis` is required. 

`sudo apt-get install ros-hydro-octovis` to install it and 
`octovis octomap_name.ot` to show the 3D Octomap.

## TODO: Using RGBD-SLAM to create an 3D OCTOMAP

The package `squirrel_rgbd_mapping` package hasn't been tested on Indigo. If you
compile on Indigo, just ignore the package (this can be done just by creating
an empty file `CATKIN_IGNORE` inside the package folder).


