cmake_minimum_required(VERSION 2.8.3)
project(squirrel_3d_localizer)

set(ROS_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

set(squirrel_3d_localizer_CATKIN_DEPENDENCIES 
  cmake_modules
  geometry_msgs
  laser_geometry
  message_filters
  message_generation
  message_runtime
  nav_msgs
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  squirrel_3d_localizer_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)

## include catkin
find_package(catkin REQUIRED COMPONENTS ${squirrel_3d_localizer_CATKIN_DEPENDENCIES})
include_directories(include ${catkin_INCLUDE_DIRS})

## Enable OpenMP support
find_package(OpenMP)
if(${OPENMP_FOUND})
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}") 
else(${OPENMP_FOUD})
  message(WARNING "OpenMP not supported. Parallelization disabled.")
endif(${OPENMP_FOUND})

## include boost random
find_package(Boost REQUIRED COMPONENTS random)

## include Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

## include octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

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

find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})                                                          
link_directories(${DYNAMICEDT3D_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})

set(squirrel_3d_localizer_LIBRARIES 
  endpoint_model
  map_model 
  motion_model 
  observation_model 
  raycasting_model
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${squirrel_3d_localizer_LIBRARIES} squirrel_3d_localizer
  CATKIN_DEPENDS ${squirrel_3d_localizer_CATKIN_DEPENDENCIES}
  DEPENDS octomap OpenMP Boost Eigen3
)

add_library(endpoint_model src/EndpointModel.cpp)

add_library(map_model src/MapModel.cpp)

add_library(motion_model src/MotionModel.cpp)

add_library(observation_model src/ObservationModel.cpp)

add_library(raycasting_model src/RaycastingModel.cpp)
target_link_libraries(raycasting_model observation_model)

add_library(squirrel_3d_localizer src/SquirrelLocalizer.cpp)
target_link_libraries(squirrel_3d_localizer ${squirrel_3d_localizer_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(squirrel_3d_localizer squirrel_3d_localizer_msgs_generate_messages_cpp)

add_executable(squirrel_3d_localizer_node src/squirrel_3d_localizer_node.cpp)
target_link_libraries(squirrel_3d_localizer_node squirrel_3d_localizer ${catkin_LIBRARIES})

# install
install(TARGETS ${LIBRARIES} squirrel_3d_localizer
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
) 

install(TARGETS squirrel_3d_localizer_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
