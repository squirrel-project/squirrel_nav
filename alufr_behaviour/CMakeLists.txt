cmake_minimum_required(VERSION 2.8.3)
project(alufr_behaviour)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  	rospy
  	std_msgs
    dynamic_reconfigure 
	alufr_mapping_msgs
	geometry_msgs
	sensor_msgs
)

catkin_python_setup()

# protected region additional user defined REQUIREMENTS on begin #
# protected region additional user defined REQUIREMENTS end #


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
  	rospy
  	std_msgs
    dynamic_reconfigure
	alufr_mapping_msgs
	geometry_msgs
	sensor_msgs
   
)
