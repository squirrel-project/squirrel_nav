cmake_minimum_required(VERSION 2.8.3)
project(squirrel_localizer)

set(ROS_BUILD_TYPE Release)

## Package dependencies
set(squirrel_localizer_DEPENDENCIES 
  geometry_msgs
  laser_geometry
  message_generation
  message_runtime
  nav_msgs
  pcl_ros
  robotino_msgs
  rosbag
  roscpp
  roslib
  rospy
  sensor_msgs
  squirrel_localizer_msgs
  std_msgs
  std_srvs
  tf
)

find_package(catkin REQUIRED 
  COMPONENTS
  ${squirrel_localizer_DEPENDENCIES}
)

find_package(PCL REQUIRED QUIET)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${squirrel_localizer_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(squirrel_localizer_localization_node
  src/BasicMotion.cpp
  src/BasicSensor.cpp
  src/CommandArgs.cpp
  src/DistanceWeight.cpp
  src/FilesysTools.cpp
  src/FloatMap.cpp
  src/Geometry.cpp
  src/KLDSampling.cpp
  src/LaserSensor2D.cpp
  src/Localization.cpp
  src/LocalizerApp.cpp
  src/OSSpecific.cpp
  src/PermutationSampler.cpp
  src/RawLaser.cpp
  src/RobotLaser.cpp
  src/RuntimeError.cpp
  src/ScanMatcher.cpp
  src/SimpleMotionModel.cpp
  src/Stat.cpp
  src/StringTools.cpp
  src/TimeUtil.cpp
)

target_link_libraries(squirrel_localizer_localization_node
  ${catkin_LIBRARIES}  
  ${PCL_LIBRARIES}
)

add_dependencies(squirrel_localizer_localization_node
  squirrel_localizer_msgs_generate_messages_cpp
  robotino_msgs_generate_messages_cpp
)

add_executable(squirrel_localizer_tf_pose_node
  src/PosePublisher.cpp
)

target_link_libraries(squirrel_localizer_tf_pose_node
  ${catkin_LIBRARIES}  
)

#############
## Install ##
############

install(TARGETS
  squirrel_localizer_localization_node
  squirrel_localizer_tf_pose_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${Project_name}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE
)











