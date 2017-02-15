cmake_minimum_required(VERSION 2.8.3)
project(squirrel_dynamic_filter)

SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  /home/dewan/software/pcl/build/)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)
#SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)
#SET(libg2o_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

set(squirrel_dynamic_filter_DEPENDENCIES
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  squirrel_dynamic_filter_msgs
  tf
  octomap_ros
  octomap_msgs
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/mlpack/src/)


MESSAGE(status "aaadadada")

MESSAGE(status ${CMAKE_BINARY_DIR})

#include_directories(${CMAKE_BINARY_DIR}/include/mlpack/)
add_subdirectory(external/mlpack)
set(mlpack_lib mlpack)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/g2o/)
add_subdirectory(external/g2o external/g2o/build)
set(G2O_CORE g2ocore stuff types_slam2d types_slam3d solver_csparse csparse_extension)
#set(PCL_DIR "/export/home/dewan/software/pcl/build/")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${squirrel_dynamic_filter_DEPENDENCIES})

set(BOOST_LIBS program_options serialization random system)
find_package(Boost REQUIRED COMPONENTS system signals thread)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
MESSAGE(${PCL_INCLUDE_DIRS})

#link_directories("/home/dewan/code/sceneflow/build/")
#link_directories("/home/dewan/code/sceneflow/build/")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/squirrel_dynamic_filter/)

# find_package(G2O REQUIRED)
# if(G2O_FOUND)
#   include_directories( ${G2O_INCLUDE_DIR})
#   link_directories(${G2O_LIBRARY_DIR})
# endif(G2O_FOUND)
# message(${G2O_INCLUDE_DIR})

## import Eigen3 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
set(CMAKE_CXX_FLAGS " -pthread -Wno-deprecated" )
	
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Armadillo 4.2 REQUIRED)

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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
##
## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES squirrel_dynamic_filter
  DEPENDS Boost PCL
  CATKIN_DEPENDS ${squirrel_dynamic_filter_DEPENDECIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_library(vertex_se3_vector3D src/vertex_se3_vector3D.cpp src/edge_unary.cpp src/edge.cpp)
target_link_libraries (vertex_se3_vector3D ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
add_dependencies(vertex_se3_vector3D ${G2O_CORE})
#add_executable(sensor_msgs_to_pcl_tf src/sensor_msgs_to_pcl_tf.cpp)
#target_link_libraries(sensor_msgs_to_pcl_tf ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})


#add_executable(dynamic_filter_node src/dynamic_filter_node.cpp)

add_executable(dynamic_filter_node src/dynamic_filter_node.cpp
               src/EstimateFeature.cpp src/EstimateCorrespondence
              src/EstimateMotion.cpp src/DynamicScore.cpp)
target_link_libraries(dynamic_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES} vertex_se3_vector3D ${G2O_CORE_LIBRARY}
                      ${G2O_TYPES_SLAM3D} ${PCL_LIBRARIES} ${G2O_STUFF_LIBRARY}
                      ${G2O_SOLVER_CSPARSE} ${CSPARSE_LIBRARY}
                      ${G2O_SOLVER_CSPARSE_EXTENSION} ${ARMADILLO_LIBRARIES}
                      ${G2O_CORE} mlpack)
add_dependencies(dynamic_filter_node
                 squirrel_dynamic_filter_msgs_generate_messages_cpp ${G2O_CORE}
                 ${mlpack_lib})

add_executable(preprocessing src/preprocessing.cpp)
target_link_libraries(preprocessing ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_CORE}) 
add_dependencies(preprocessing squirrel_dynamic_filter_msgs_generate_messages_cpp ${G2O_CORE})

add_executable(l_frequency src/l_frequency.cpp)
target_link_libraries(l_frequency ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${G2O_CORE})
add_dependencies(l_frequency squirrel_dynamic_filter_msgs_generate_messages_cpp)
#add_executable(test_odometry src/test_odometry)
#target_link_libraries(test_odometry ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} vertex_se3_vector3D ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${PCL_LIBRARIES} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${CSPARSE_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION})

#add_executable(visualize_intensity src/visalize_intensity.cpp)
#target_link_libraries(visualize_intensity ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_executable(remove_dynamic src/remove_dynamic.cpp)
target_link_libraries(remove_dynamic ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${G2O_CORE})
add_dependencies(remove_dynamic ${G2O_CORE})

add_executable(temporal_inference src/TemporalInference.cpp)
target_link_libraries(temporal_inference ${catkin_LIBRARIES} ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES} mlpack "/usr/lib/libarmadillo.so" vertex_se3_vector3D ${G2O_CORE})
add_dependencies(temporal_inference ${G2O_CORE} ${mlpack_lib})

add_executable(sensor_to_base_link src/sensor_to_base_link.cpp)
target_link_libraries(sensor_to_base_link ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${G2O_CORE})
add_dependencies(sensor_to_base_link ${G2O_CORE})

#add_executable(publish_color_octomap src/publish_color_octomap.cpp)
#target_link_libraries(publish_color_octomap ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES}
#${G2O_CORE})

#add_executable(octomap_to_cloud src/octomap_to_cloud.cpp)
#target_link_libraries(octomap_to_cloud ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(writeOctree src/writeOctree.cpp)
#target_link_libraries(writeOctree ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(color_octomap src/color_octomap.cpp)
#target_link_libraries(color_octomap ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})




#add_executable(store_cloud src/store_cloud.cpp)
#target_link_libraries(store_cloud ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(static_classify src/StaticClassifyOctoMap.cpp)
target_link_libraries(static_classify ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_CORE})
add_dependencies(static_classify squirrel_dynamic_filter_msgs_generate_messages_cpp ${G2O_CORE})




## Declare a C++ library
# add_library(dynamic_filter
#   src/${PROJECT_NAME}/dynamic_filter.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(dynamic_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(dynamic_filter_node src/dynamic_filter_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(dynamic_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(dynamic_filter_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS dynamic_filter dynamic_filter_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dynamic_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
