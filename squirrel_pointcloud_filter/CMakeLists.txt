cmake_minimum_required(VERSION 2.8.3)
project(squirrel_pointcloud_filter)

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
set(squirrel_pointcloud_filter_DEPENDENCIES
  pcl_conversions pcl_ros roscpp)

## Import ROS dependencies
find_package(catkin REQUIRED COMPONENTS ${squirrel_pointcloud_filter_DEPENDENCIES})
include_directories(${catkin_INCLUDE_DIRS})

## Catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES squirrel_pointcloud_filter_node
 CATKIN_DEPENDS ${squirrel_pointcloud_filter_DEPENDENCIES}
#  DEPENDS system_lib
)

## Build the pointcloud filter.
include_directories(include)
add_executable(squirrel_pointcloud_filter_node 
  src/pointcloud_filter_node.cpp src/pointcloud_filter.cpp)
target_link_libraries(squirrel_pointcloud_filter_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS squirrel_pointcloud_filter squirrel_pointcloud_filter_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_squirrel_pointcloud_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
