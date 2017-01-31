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

#set(PCL_DIR "/export/home/dewan/software/pcl/build/")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${squirrel_dynamic_filter_DEPENDENCIES}
)

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

find_package(G2O REQUIRED)
if(G2O_FOUND)
  include_directories( ${G2O_INCLUDE_DIR})
  link_directories(${G2O_LIBRARY_DIR})
endif(G2O_FOUND)
MESSAGE(${G2O_INCLUDE_DIR})

## Import Eigen3 
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
  ${ARMADILLO_INCLUDE_DIRS}
)

add_custom_target(build_g2o
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/lib/libg2o_core.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
	COMMENT "copy g2o"
)

add_library(g2o SHARED)
set_target_properties(g2o PROPERTIES LINKER_LANGUAGE CXX) 
add_dependencies(g2o build_g2o)

add_custom_target(build_mlpack
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/lib/libmlpack.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
	COMMENT "copy libmlpack"
)

add_library(mlpack SHARED)
set_target_properties(mlpack PROPERTIES LINKER_LANGUAGE CXX) 
add_dependencies(mlpack build_mlpack)

add_library(vertex_se3_vector3D src/vertex_se3_vector3D.cpp src/edge_unary.cpp src/edge.cpp)
TARGET_LINK_LIBRARIES (vertex_se3_vector3D ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
#add_executable(sensor_msgs_to_pcl_tf src/sensor_msgs_to_pcl_tf.cpp)
#target_link_libraries(sensor_msgs_to_pcl_tf ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})


#add_executable(dynamic_filter_node src/dynamic_filter_node.cpp)

add_executable(dynamic_filter_node src/dynamic_filter_node.cpp
               src/EstimateFeature.cpp src/EstimateCorrespondence
              src/EstimateMotion.cpp src/DynamicScore.cpp)
target_link_libraries(dynamic_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} vertex_se3_vector3D ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${PCL_LIBRARIES} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${CSPARSE_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libmlpack.so ${ARMADILLO_LIBRARIES})
add_dependencies(dynamic_filter_node squirrel_dynamic_filter_msgs_message_generates_cpp mlpack g2o)

add_executable(preprocessing src/preprocessing.cpp)
target_link_libraries(preprocessing ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
add_dependencies(preprocessing squirrel_dynamic_filter_msgs_generate_messages_cpp g2o)

add_executable(l_frequency src/l_frequency.cpp)
target_link_libraries(l_frequency ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(l_frequency squirrel_dynamic_filter_msgs_generate_messages_cpp)
#add_executable(test_odometry src/test_odometry)
#target_link_libraries(test_odometry ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} vertex_se3_vector3D ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${PCL_LIBRARIES} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${CSPARSE_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION})

#add_executable(visualize_intensity src/visalize_intensity.cpp)
#target_link_libraries(visualize_intensity ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_executable(remove_dynamic src/remove_dynamic.cpp)
target_link_libraries(remove_dynamic ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(temporal_inference src/TemporalInference.cpp)
target_link_libraries(temporal_inference ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libmlpack.so "/usr/lib/libarmadillo.so" vertex_se3_vector3D ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${PCL_LIBRARIES} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${CSPARSE_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION})

add_executable(sensor_to_base_link src/sensor_to_base_link.cpp)
target_link_libraries(sensor_to_base_link ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})


#add_executable(store_cloud src/store_cloud.cpp)
#target_link_libraries(store_cloud ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(static_classify src/StaticClassifyOctoMap.cpp)
target_link_libraries(static_classify ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
add_dependencies(static_classify squirrel_dynamic_filter_msgs_generate_messages_cpp g2o)


#############
## Install ##
#############


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

