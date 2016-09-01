cmake_minimum_required(VERSION 2.8.3)
project(squirrel_3d_localizer)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_conversions pcl_ros message_filters sensor_msgs cmake_modules)
find_package(OpenMP)
find_package(Boost REQUIRED COMPONENTS random)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

# required for OpenMP
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}") 

set(LIBRARIES map_model motion_model observation_model raycasting_model)

find_package(dynamicEDT3D)
if (${dynamicEDT3D_FOUND})                                                                       
  include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})                                                          
  link_directories(${DYNAMICEDT3D_LIBRARY_DIRS})
  LINK_LIBRARIES(${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})
  list(APPEND LIBRARIES endpoint_model)
else (${dynamicEDT3D_FOUND})
  message(WARNING "dynamicEDT3D library (part of OctoMap >1.5) not found, skipping endpoint model")
  add_definitions(-DSKIP_ENDPOINT_MODEL)
endif (${dynamicEDT3D_FOUND})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES} squirrel_3d_localizer
  CATKIN_DEPENDS tf
  DEPENDS octomap OpenMP
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

if (${dynamicEDT3D_FOUND})
  add_library(endpoint_model src/EndpointModel.cpp)
endif (${dynamicEDT3D_FOUND})

add_library(map_model src/MapModel.cpp)
add_library(motion_model src/MotionModel.cpp)
add_library(observation_model src/ObservationModel.cpp)

add_library(raycasting_model src/RaycastingModel.cpp)
target_link_libraries(raycasting_model observation_model)

add_library(squirrel_3d_localizer src/SquirrelLocalizer.cpp)
target_link_libraries(squirrel_3d_localizer ${LIBRARIES} ${catkin_LIBRARIES})

add_executable(squirrel_3d_localizer_node src/squirrel_3d_localizer_node.cpp)
target_link_libraries(squirrel_3d_localizer_node squirrel_3d_localizer ${catkin_LIBRARIES})

# install
install(TARGETS ${LIBRARIES} squirrel_3d_localizer
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
) 

install(TARGETS squirrel_3d_localizer_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
