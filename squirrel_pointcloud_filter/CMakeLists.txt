cmake_minimum_required(VERSION 2.8.3)
project(squirrel_pointcloud_filter)

## Set build type.
set(ROS_BUILD_TYPE Release)

## Enable C++11 support
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

## Set ROS dependencies
set(${PROJECT_NAME}_DEPENDENCIES
  dynamic_reconfigure 
  pcl_conversions 
  pcl_ros 
  roscpp)

## Import ROS dependencies
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_DEPENDENCIES})
include_directories(${catkin_INCLUDE_DIRS})

## Generate reconfigure files.
generate_dynamic_reconfigure_options(cfg/PointCloudFilter.cfg)

## Catkin package
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDENCIES})

## Build the pointcloud filter.
include_directories(include)
add_executable(${PROJECT_NAME}_node 
  src/pointcloud_filter.cpp  
  src/pointcloud_filter_node.cpp)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_gencfg) 

## Install
install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE)

install(
  DIRECTORY rviz cfg config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Testing.
# catkin_add_gtest(${PROJECT_NAME}-test test/test_squirrel_pointcloud_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
