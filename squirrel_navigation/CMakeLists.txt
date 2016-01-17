cmake_minimum_required(VERSION 2.8.3)
project(squirrel_navigation)

set(ROS_BUILD_TYPE Release)

## Package dependencies
set(squirrel_navigation_DEPENDENCIES
  actionlib
  base_local_planner
  clear_costmap_recovery
  costmap_2d
  dynamic_reconfigure
  dynamixel_msgs
  geometry_msgs
  message_generation
  move_base_msgs
  nav_core
  nav_msgs
  navfn
  octomap_msgs
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  rotate_recovery
  std_msgs
  std_srvs
  tf
  visualization_msgs
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(SBPL REQUIRED 
  sbpl
)

find_library(SBPL_LIBRARY 
  NAMES sbpl 
  PATHS ${SBPL_LIBRARY_DIRS} 
  NO_DEFAULT_PATH
)

set(SBPL_LIBRARIES ${SBPL_LIBRARY})

############
## catkin ##
############

find_package(catkin REQUIRED
  COMPONENTS
  ${squirrel_navigation_DEPENDENCIES}
)

generate_dynamic_reconfigure_options(
  cfg/DownprojectionLayerPlugin.cfg
  cfg/DownprojectionMultilayerPlugin.cfg
  cfg/MapLayerPlugin.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sqiurrel_planners squirrel_costmap_layers
  CATKIN_DEPENDS ${squirrel_navigation_DEPENDENCIES}
  DEPENDS SBPL
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS} 
  ${SBPL_LIBRARY_DIRS}
)

## Compiling and linking the plugins
add_library(squirrel_planners
  src/LocalPlanner.cpp
  src/GlobalPlanner.cpp
  src/LatticeSCQ.cpp
)

add_dependencies(squirrel_planners
  squirrel_nav_msgs_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

add_library(squirrel_costmap_layers
  src/MapLayer.cpp
  src/FootprintLayer.cpp
  src/InflatedLayer.cpp
  src/DownprojectionLayer.cpp
  src/DownprojectionMultilayer.cpp
  src/MultiInflatedLayer.cpp
  src/JointHandle.cpp
)

add_dependencies(squirrel_costmap_layers
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(squirrel_costmap_layers
  ${catkin_LIBRARIES}
)

## Compiling and linking the nodes 
add_executable(pointcloud_filter_node
  src/PointCloudFilterNode.cpp
  src/PointCloudFilter.cpp
)

add_executable(autolocalization_node
  src/AutolocalizationNode.cpp
  src/Autolocalization.cpp
)

target_link_libraries(autolocalization_node
  ${catkin_LIBRARIES}
)

target_link_libraries(pointcloud_filter_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS  
  squirrel_planners 
  squirrel_costmap_layers 
  pointcloud_filter_node 
  autolocalization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY 
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE
)

install(FILES 
  SquirrelLocalPlanner.xml 
  SquirrelCostmapLayers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_alufr_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
