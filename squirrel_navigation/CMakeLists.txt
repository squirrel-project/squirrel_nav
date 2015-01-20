cmake_minimum_required(VERSION 2.8.3)
project(squirrel_navigation)

## Dependencies
set(squirrel_navigation_DEPENDENCIES
  actionlib
  # cmake_modules
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  message_generation
  move_base_msgs
  nav_core
  nav_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
  octomap_msgs
  base_local_planner
  clear_costmap_recovery
  navfn
  pcl_conversions
  rotate_recovery
  pcl_ros
  roscpp
  pluginlib
  octomap_msgs
)

############
## catkin ##
############

find_package(catkin REQUIRED
  COMPONENTS
  ${squirrel_navigation_DEPENDENCIES}
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sqiurrel_local_planner squirrel_obstacles_layer
  CATKIN_DEPENDS ${squirrel_navigation_DEPENDENCIES}
  #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Compiling and linking the plugins
add_library(squirrel_local_planner
  src/LocalPlanner.cpp
)

add_library(squirrel_obstacles_layer
  src/ObstaclesLayer.cpp
)

target_link_libraries(squirrel_obstacles_layer
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
  squirrel_local_planner
  squirrel_obstacles_layer
  pointcloud_filter_node
  autolocalization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE
)

install(FILES plugins/SquirrelLocalPlanner.xml plugins/SquirrelObstaclesLayer.xml
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
