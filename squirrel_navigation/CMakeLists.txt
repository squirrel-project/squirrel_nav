cmake_minimum_required(VERSION 2.8.3)
project(squirrel_navigation)

set(ROS_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(CGAL REQUIRED)

pkg_check_modules(SBPL REQUIRED sbpl)

find_library(SBPL_LIBRARY 
  NAMES sbpl 
  PATHS ${SBPL_LIBRARY_DIRS} 
  NO_DEFAULT_PATH
)

set(SBPL_LIBRARIES ${SBPL_LIBRARY})

############
## catkin ##
############

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
  squirrel_navigation_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
  ${squirrel_navigation_DEPENDENCIES}
)

generate_dynamic_reconfigure_options(
  cfg/DownprojectionLayerPlugin.cfg
  cfg/DownprojectionMultilayerPlugin.cfg
  cfg/InflationLayerPlugin.cfg
  cfg/MapLayerPlugin.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES squirrel_costmap_layers squirrel_planners
  CATKIN_DEPENDS ${squirrel_navigation_DEPENDENCIES}
  DEPENDS SBPL
)

###########
## Build ##
###########

include(${CGAL_USE_FILE})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS} 
  ${SBPL_LIBRARY_DIRS}
)

## Compiling and linking the plugins
add_library(squirrel_planners
  src/GlobalPlanner.cpp
  src/LatticeSCQ.cpp
  src/LocalPlanner.cpp
  src/TrajectoryPlanner.cpp
  src/ControllerPID.cpp
)

add_dependencies(squirrel_planners
  squirrel_navigation_msgs_generate_messages_cpp
)

target_link_libraries(squirrel_planners
  ${catkin_LIBRARIES}
  ${SBPL_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(squirrel_costmap_layers
  src/CostmapUpdateHandle.cpp
  src/DownprojectionLayer.cpp
  src/DownprojectionMultilayer.cpp
  src/FootprintLayer.cpp
  src/FootprintMultilayer.cpp
  src/InflatedLayer.cpp
  src/InflationLayer.cpp
  src/JointHandle.cpp
  src/MapLayer.cpp
  src/MultiInflatedLayer.cpp
)

add_dependencies(squirrel_costmap_layers
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(squirrel_costmap_layers
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Compiling and linking the nodes 
add_executable(pointcloud_filter_node
  src/PointCloudFilter.cpp
  src/PointCloudFilterNode.cpp
)

target_link_libraries(pointcloud_filter_node
  ${catkin_LIBRARIES}
)

add_executable(planner_update_handle_node
  src/PlannerUpdateHandle.cpp
  src/PlannerUpdateHandleNode.cpp
)

add_dependencies(planner_update_handle_node
  squirrel_navigation_msgs_generate_messages_cpp
)

target_link_libraries(planner_update_handle_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS  
  planner_update_handle_node
  pointcloud_filter_node   
  squirrel_costmap_layers
  squirrel_planners 
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
  SquirrelCostmapLayers.xml
  SquirrelPlanners.xml 
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
