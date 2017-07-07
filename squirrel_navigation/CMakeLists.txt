cmake_minimum_required(VERSION 2.8.3)
project(MAP_localizer)

## Enable C++11 support.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 " 
    "support. Please use a different C++ compiler.")
endif()

## Set Release as default build mode.
set(ROS_BUILD_TYPE Release)

## Import external modules.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake_modules)

## Import Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Import OMPL
find_package(OMPL REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

## Import ros dependencies.
set(squirrel_navigation_catkin_DEPENDENCIES
  actionlib costmap_2d base_local_planner clear_costmap_recovery dynamic_reconfigure 
  geometry_msgs message_generation message_runtime move_base_msgs nav_core nav_msgs navfn 
  pluginlib roscpp squirrel_navigation_msgs std_msgs std_srvs tf visualization_msgs)
find_package(catkin REQUIRED COMPONENTS ${squirrel_navigation_catkin_DEPENDENCIES})
include(${catkin_INCLUDE_DIRS})

## Generate reconfig files.
generate_dynamic_reconfigure_options(
  cfg/utils/alpha_beta_filter.cfg cfg/safety/scan_observer.cfg
  cft/safety/arm_skin_observer.cfg cfg/controller_pid.cfg cfg/footprint_planner.cfg 
  cfg/global_planner.cfg cfg/global_planner.cfg cfg/local_planner.cfg 
  cfg/motion_planner.cfg cfg/navigation_layer.cfg)

## Build libraries.
add_library(squirrel_navigation_utils src/utils/alpha_beta_filter.cpp)
target_link_libraries(squirreL_navigation_utils ${catkin_LIBRARIES})
add_dependencies(squirrel_navigation_utils squirrel_navigation_gencfg)

add_library(squirrel_navigation_safety 
  src/safety/scan_observer.cpp src/safety/arm_skin_observer.cpp)
target_link_libraries(squirrel_navigtaion_safety
  ${catkin_LIBRARIES} squirrel_navigation_utils)
add_dependencies(squirrel_navigation_safety squirrel_navigation_gencfg)

add_library(squirrel_navigation_planners 
  src/local_planner.cpp src/motion_planner.cpp src/controller_pid.cpp 
  src/global_planner.cpp src/footprint_planner.cpp)
target_link_libraries(squirrel_navigation_planners 
  ${catkin_LIBRARIES} ${OMPL_LIBRARIES} squirrel_navigation_safety)
add_dependencies(squirrel_navigation_planners squirrel_navigation_gencfg)

add_library(squirrel_navigation_costmap_layer
  src/navigation_layer.cpp external/costmap_2d_strip/obstacle_layer.cpp
  external/costmap_2d_strip/static_layer.cpp 
  external/costmap_2d_strip/voxel_layer.cpp)
target_link_libraries(squirrel_navigation_costmap_navigation_layer 
  ${catkin_LIBRARIES})
add_dependencies(squirrel_navigation_costmap_layer squirrel_navigation_gencfg)

## Built libraries.
set(squirrel_navigation_LIBRARIES 
  squirrel_navigation_utils squirrel_navigation_safety 
  squirrel_navigation_planners squirrel_navigation_costmap_layer)

## Catkin package exports.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${squirrel_navigation_LIBRARIES}
  CATKIN_DEPENDS ${squirrel_navigation_catkin_DEPENDENCIES}
  DEPENDS OMPL Eigen3)

## Install.
install(
  TARGETS ${squirrel_navigation_PLANNERS} ${squirrel_navigation_COSTMAP_LAYERS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE)

install(
  DIRECTORY config/robotino
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(
  FILES squirrel_navigation_costmap_layers.xml squirrel_navigation_planners.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Testing 
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_alufr_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
