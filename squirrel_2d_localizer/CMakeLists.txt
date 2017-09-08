cmake_minimum_required(VERSION 2.8.3)
project(squirrel_2d_localizer)

## Set build type.
set(ROS_BUILD_TYPE Release)

## Import external modules.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

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

## Enable OpenMP parallelization for gcc.
find_package(OpenMP)
if(OPENMP_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)

## Import Eigen3.
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Set ROS dependencies
set(${PROJECT_NAME}_catkin_DEPENDENCIES
  dynamic_reconfigure
  geometry_msgs 
  message_filters 
  nav_msgs 
  roscpp 
  roslib 
  sensor_msgs
  squirrel_2d_localizer_msgs 
  tf)

## Import ROS dependencies
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_catkin_DEPENDENCIES})
include_directories(${catkin_INCLUDE_DIRS})

## Genereate reconfig files.
generate_dynamic_reconfigure_options(
  cfg/LaserModel.cfg
  cfg/MonteCarloLocalization.cfg
  cfg/MotionModel.cfg
  cfg/TwistCorrection.cfg)

## Catkin package.
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${${PROJECT_NAME}_catkin_DEPENDENCIES}
 DEPENDS Eigen3)

## Building the localizer.
set(ROS_BUILD_TYPE Release)
include_directories(include)
add_executable(${PROJECT_NAME}_node  
  src/convolution.cpp 
  src/grid_map.cpp 
  src/laser_model.cpp 
  src/laser_model_ros.cpp
  src/latent_model_likelihood_field.cpp
  src/latent_model_likelihood_field_ros.cpp
  src/localizer.cpp 
  src/localizer_ros.cpp 
  src/localizer_ros_node.cpp 
  src/motion_model.cpp
  src/motion_model_ros.cpp
  src/resampling.cpp 
  src/se2_types.cpp 
  src/extras/twist_correction.cpp 
  src/extras/twist_correction_ros.cpp)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_gencfg
  squirrel_2d_localizer_msgs_generate_messages_cpp)

## Installation.
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_squirrel_2d_localizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
