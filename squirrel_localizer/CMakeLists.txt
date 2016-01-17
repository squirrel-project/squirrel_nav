cmake_minimum_required(VERSION 2.8.3)
project(squirrel_localizer)

set(ROS_BUILD_TYPE Release)

## Package dependencies
set(squirrel_localizer_DEPENDENCIES
  roslib
  roscpp
  rospy
  tf
  std_msgs
  message_generation
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  laser_geometry
  pcl_ros
  rosbag
  robotino_msgs
  message_runtime
  message_generation
  squirrel_localizer_msgs
)

find_package(catkin REQUIRED 
  COMPONENTS
  ${squirrel_localizer_DEPENDENCIES}
)

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
)

add_executable(squirrel_localizer_localization_node
  src/LocalizerApp.cpp
  src/Localization.cpp
  src/RawLaser.cpp
  src/RobotLaser.cpp
  src/Geometry.cpp
  src/DistanceWeight.cpp
  src/BasicMotion.cpp
  src/SimpleMotionModel.cpp
  src/BasicSensor.cpp
  src/LaserSensor2D.cpp
  src/PermutationSampler.cpp
  src/Stat.cpp
  src/CommandArgs.cpp
  src/OSSpecific.cpp
  src/FilesysTools.cpp
  src/RuntimeError.cpp
  src/StringTools.cpp
  src/TimeUtil.cpp
  src/KLDSampling.cpp
  src/ScanMatcher.cpp
  src/FloatMap.cpp
)

target_link_libraries(squirrel_localizer_localization_node
  ${catkin_LIBRARIES}  
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











