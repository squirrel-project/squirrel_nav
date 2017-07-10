cmake_minimum_required(VERSION 2.8.3)
project(squirrel_footprint_observer)

set(ROS_BUILD_TYPE Release)

## Import cmake modules.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

## Set ROS dependencies.
set(squirrel_footprint_observer_DEPENDENCIES
  dynamic_reconfigure geometry_msgs message_runtime message_generation 
  roscpp tf squirrel_footprint_observer_msgs std_msgs)

## Import ROS dependencies.
find_package(catkin REQUIRED COMPONENTS ${squirrel_footprint_observer_DEPENDENCIES})
include_directories(${catkin_INCLUDE_DIRS})

## Import CGAL
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

## Enable C++11 support (after CGAL!).
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

## Import yaml-cpp
find_package(YamlCpp REQUIRED)
include_directories(${YAMLCPP_INCLUDE_DIR})

## Genereate reconfig files.
generate_dynamic_reconfigure_options(cfg/footprint_observer.cfg)

## Building the footprint observer
include_directories(include)
add_executable(squirrel_footprint_observer_node
  src/footprint_observer_node.cpp src/footprint_observer.cpp)
target_link_libraries(squirrel_footprint_observer_node 
  ${catkin_LIBRARIES} ${YAMLCPP_LIBRARY})
add_dependencies(squirrel_footprint_observer_node 
  squirrel_footprint_observer_msgs_generate_messages_cpp
  squirrel_footprint_observer_gencfg)

## Catkin package.
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES squirrel_footprint_observer_node
 CATKIN_DEPENDS ${squirrel_footprint_observer_DEPENDENCIES}
 DEPENDS CGAL YamlCpp)

## Install
install(TARGETS squirrel_footprint_observer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE)

install(FILES
  config/robotino_footprint.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_squirrel_footprint_observer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
