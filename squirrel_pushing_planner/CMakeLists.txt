cmake_minimum_required(VERSION 2.8.3)
project(squirrel_pushing_planner)

set(ROS_BUILD_TYPE Release)

## Package dependencies
set(squirrel_pushing_planner_DEPENDENCIES
  geometry_msgs
  nav_msgs
  roscpp
  squirrel_rgbd_mapping_msgs
  tf
)


find_package(catkin REQUIRED 
  COMPONENTS
  ${squirrel_pushing_planner_DEPENDENCIES}
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${squirrel_pushing_planner_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pushing_planner_node
  src/PushingPlanner.cpp
  src/PushingPlannerNode.cpp
)

add_dependencies(pushing_planner_node 
  squirrel_rgbd_mapping_msgs_generate_messages_cpp  
)

target_link_libraries(pushing_planner_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS pushing_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_squirrel_pushing_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
