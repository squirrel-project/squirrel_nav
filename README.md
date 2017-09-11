squirrel_nav
============

Repository for navigation related SQUIRREL packages.

## Requirement

Install the package dependencies running
```bash
rosdep install --from-path squirrel_nav -i -y
sudo apt-get install ros-indigo-sbpl
```

In order to properly generate the dynamic reconfiguration files,
setting execute permission to the files contained in `cfg` folder
might be needed: run
```bash
chmod a+x cfg/*	
```
from the home folder of the package.

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

An rviz configuration file for visualising the current map generated
can be found in the git repository IN
*squirrel_navigation/config/robot1/rviz/rviz_hydro_gmapping.rviz*.
You should see a small part of a map.  Move the robotino with the
joystick to complete the map.  If done, save the map with the
following command:

```bash 
$ rosrun map_server map_saver -f [mymap] 
``` 

where *[mymap]* should be replaced by the correct path and filename.
`mymap.pgm` and `mymap.yaml` are created in the folder where the above
command has been executed.

Move the map files into the folder
*navigation/squirrel_navigation/maps* and create a symbolic link to
the map:
```bash
$ roscd squirrel_navigation
$ cd maps
$ rm default-map.yaml
$ ln -s mymap.yaml default-map.yaml
```
Be sure that both the `.yaml` file and `.pgm` files are stored
in the same folder.

## Starting The Navigation

```bash
$ roslaunch squirrel_navigation navigation.launch {map_file:=[mymap.yaml]}
```

The argument `map_file:=[mymap.yaml]` can be omitted and the map
pointed by `default-map.yaml` is loaded. In case the map's file is
specified, *[mymap.yaml]* has to be replaced by the recorded mapping
file.  You have to use the full path to the mapping file, even if the
yaml file is in the current working directory.

### Localising in a 2D Gridmap

Set at first an estimated pose with the button *2D Pose Estimate* and
move the robot around the map to localise the robot. To start the
robot in the last pose, set `use_last_pose` parameter to `true` in
[squirrel_2d_localizer](https://github.com/squirrel-project/squirrel_nav/tree/indigo_dev/squirrel_2d_localizer)
package.

### Performing a navigation task

After that, you can send a navigation goal via the rviz button *2D Nav Goal*.
The robot should now move to the goal.

### Notes on navigation

The setup is meant for a robot provided with 
- A laser rangefinder, used for localisation and obstacles mapping
- A depth camera, used for obstacles mapping and 3D navigation

