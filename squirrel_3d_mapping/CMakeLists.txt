cmake_minimum_required(VERSION 2.8)
project(squirrel_3d_mapping)

set(ROS_BUILD_TYPE Release)

## Package dependencies
set(squirrel_3d_mapping_DEPENDENCIES
  dynamic_reconfigure
  geometry_msgs
  message_generation 
  message_runtime
  nav_msgs
  nodelet
  octomap_msgs  
  octomap_ros
  pcl_conversions 
  pcl_ros
  roscpp
  sensor_msgs
  squirrel_3d_mapping_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

############
## catkin ##
############

find_package(catkin REQUIRED 
  COMPONENTS 
  ${squirrel_3d_mapping_DEPENDENCIES}
)

#############
##   PCL   ##
#############

find_package(PCL REQUIRED 
  QUIET COMPONENTS 
  common 
  filters
  io 
  sample_consensus 
  segmentation 
)

#############
## OctoMap ##
#############

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${DYNAMIC_EDT_3D_DIRS}
)

generate_dynamic_reconfigure_options(cfg/OctomapServer.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${squirrel_3d_mapping_DEPENDENCIES}
  DEPENDS octomap PCL
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(dynamic_edt src/DynamicEDT3D.cpp src/DynamicEDTOctomap.cpp)
target_link_libraries(dynamic_edt ${LINK_LIBS})

add_dependencies(dynamic_edt squirrel_3d_mapping_msgs_generate_messages_cpp)

add_library(${PROJECT_NAME} src/OctomapServer.cpp src/OctomapServerMultilayer.cpp src/TrackingOctomapServer.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} squirrel_3d_mapping_msgs_generate_messages_cpp)

add_executable(octomap_server_node src/OctomapServerNode.cpp)
target_link_libraries(octomap_server_node ${PROJECT_NAME} ${LINK_LIBS} dynamic_edt)

add_executable(octomap_server_static src/OctomapServerStatic.cpp)
target_link_libraries(octomap_server_static ${PROJECT_NAME} ${LINK_LIBS} dynamic_edt)

add_executable(octomap_server_multilayer src/OctomapServerMultilayerNode.cpp)
target_link_libraries(octomap_server_multilayer ${PROJECT_NAME} ${LINK_LIBS} dynamic_edt)

add_executable(octomap_saver src/OctomapSaver.cpp)
target_link_libraries(octomap_saver ${PROJECT_NAME} ${LINK_LIBS} dynamic_edt)

add_executable(octomap_tracking_server_node src/OctomapTrackingServerNode.cpp)
target_link_libraries(octomap_tracking_server_node ${PROJECT_NAME} ${LINK_LIBS} dynamic_edt)

add_executable(publish_color_octomap src/PublishColorOctomap.cpp)
target_link_libraries(publish_color_octomap ${LINK_LIBS})


## Nodelet
add_library(octomap_server_nodelet src/OctomapServerNodelet.cpp)
target_link_libraries(octomap_server_nodelet ${PROJECT_NAME} ${LINK_LIBS} dynamic_edt)

#############
## Install ##
#############

install(TARGETS
  dynamic_edt
  octomap_saver
  octomap_server_multilayer
  octomap_server_node
  octomap_server_nodelet
  octomap_server_static
  octomap_tracking_server_node
  squirrel_3d_mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
