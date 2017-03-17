cmake_minimum_required(VERSION 2.8.3)
project(squirrel_footprint_observer)

set(ROS_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

## Set ROS dependencies.
set(squirrel_footprint_observer_DEPENDENCIES
  geometry_msgs message_runtime message_generation roscpp 
  tf squirrel_footprint_observer_msgs std_msgs)

## Import ROS dependencies.
find_package(catkin REQUIRED COMPONENTS ${squirrel_footprint_observer_DEPENDENCIES})
include_directories(${catkin_INCLUDE_DIRS})

## Import CGAL
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

## Import yaml-cpp
find_package(YamlCpp REQUIRED)
include_directories(${YAMLCPP_INCLUDE_DIR})

## Catkin package.
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES squirrel_footprint_observer_node
 CATKIN_DEPENDS ${squirrel_footprint_observer_DEPENDENCIES}
 DEPENDS CGAL
)

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

## Building the footprint observer
include_directories(include)
add_executable(squirrel_footprint_observer_node
  src/footprint_observer_node.cpp src/footprint_observer.cpp)
target_link_libraries(squirrel_footprint_observer_node 
  ${catkin_LIBRARIES} ${YAMLCPP_LIBRARY})
add_dependencies(squirrel_footprint_observer_node 
  squirrel_footprint_observer_msgs_generate_messages_cpp)

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_squirrel_footprint_observer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
