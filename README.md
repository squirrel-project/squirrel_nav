navigation
==========

Repository for navigation related SQUIRREL packages.

## Creating a 2D Gridmap

Run the mapping routine by executing

```bash
roslaunch robotino_navigation gmapping.launch 
``` 

The map created can be then visualised in RViz. The robot should be
teleoperated to explore the area. Once the map of the scene is
correctly computed, it can be stored running

```bash
rosrun map_server map_saver -f [map_name]
```

`map_name.pgm` and `map_name.yaml` are created in the folder where the
above command has been executed.

## Starting Laser Based Navigation in the 2D Map

```bash
roslaunch robotino_navigation navigation.launch map_file:=[/full/path/to/mapping.yaml]
```
*/full/path/to/mapping.yaml* has to be replaced by the recorded
mapping file (see \ref{mapping}). You have to use the full path to the mapping file,
even if the yaml file is in the same directory.

To set a initial pose estimation, start rviz on your desktop.
```bash
rosrun rviz rviz
```
There is rviz configuration files in our git repository *alufr_navigation/config*,
that can be loaded with *Ctrl+O*.
Set at first a pose estimate with the button *2D Pose Estimate*
and move the robot around the map to localise the
robot. After that, you can send a navigation goal via the rviz button *2D Nav Goal*.
The robot should now move to the goal.


## Creating a 3D Octomap Localising in a 2D Map

To generate a 3D map of scene the pose of the robot must be known. For this, perform the
localisation procedure as described in the section above. Once the robot is localised, 
launch *octomap_server*:

```bash
roslaunch robotino_navigation octomap_server.launch
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


## Using RGBD-SLAM to create an 3D OCTOMAP



