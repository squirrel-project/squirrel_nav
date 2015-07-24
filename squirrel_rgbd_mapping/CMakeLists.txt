cmake_minimum_required(VERSION 2.4.6)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# #For annotated profiling with perf: 
SET(CMAKE_CXX_FLAGS "-ggdb -O3 -fPIC -std=c++0x")

####################################################
# ROS
####################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

project(squirrel_rgbd_mapping)


set(catkin_RUN_PACKAGES 
  roscpp
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  visualization_msgs
  squirrel_rgbd_mapping_msgs
  pcl_ros # will automatically pull in all PCL dependecies
  pcl_conversions
  rosbag 
  rosconsole
  message_filters
  cv_bridge
  image_transport 
  tf  
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
	cmake_modules # Eigen
)

find_package(catkin REQUIRED COMPONENTS 
  ${catkin_BUILD_PACKAGES}
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

link_libraries(rgbdslam 
  ${catkin_LIBRARIES}
)

####################################################
# Required 3rd party
####################################################
# PCL #######################
# used pcl_ros above
#find_package(PCL 1.7 REQUIRED COMPONENTS common io)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# Octomap ###################
find_package(octomap 1.6 REQUIRED)
include_directories(BEFORE ${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

# g2o #######################
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
link_libraries(${G2O_LIBRARIES})
message("G2o-libraries " ${G2O_LIBRARIES})

# Qt ########################
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

# OpenMP ########################
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
# This is necessary as all ui files etc will get dumped in the bottom of then binary directory. 
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_QTOPENGL_INCLUDE_DIR}) #${CMAKE_CURRENT_SOURCE_DIR}/external/gicp/ann_1.1.2/include/ANN)
 	
link_directories(${LIBRARY_OUTPUT_PATH})

# set(${CMAKE_LIBRARY_PATH} ${LIBRARY_OUTPUT_PATH})


#Only possible when not subscribing to point cloud topic. Possibly dangerous anyway, but halves storage space per point cloud
#ADD_DEFINITIONS(-DRGB_IS_4TH_DIM) 
#ADD_DEFINITIONS(-DDO_FEATURE_OPTIMIZATION)
#ADD_DEFINITIONS(-DHEMACLOUDS) 
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)

#############################
# OpenCV ####################
#############################
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})


#############################
# Eigen ####################
#############################
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#############################
# Suitesparse ####################
#############################
find_package(SuiteSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
link_directories(${CSPARSE_LIBRARY})

##############################################################################
# Some QT Magic (MOC Creation)
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp qtros.h openni_listener.h qt_gui.h graph_manager.h glviewer.h ros_service_ui.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources to Compile
##############################################################################
set(ADDITIONAL_SOURCES src/gicp-fallback.cpp src/main.cpp src/qtros.cpp src/openni_listener.cpp src/qt_gui.cpp src/node.cpp src/glviewer.cpp src/parameter_server.cpp src/ros_service_ui.cpp src/misc.cpp src/landmark.cpp src/loop_closing.cpp src/ColorOctomapServer.cpp src/scoped_timer.cpp src/icp.cpp src/matching_result.cpp)
#GraphManager Files
set(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/graph_manager.cpp src/graph_mgr_io.cpp src/graph_manager2.cpp src/graph_mgr_odom.cpp)
set(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/transformation_estimation_euclidean.cpp src/transformation_estimation.cpp src/graph_manager2.cpp src/header.cpp)
#Computation of meaningful covariances
set(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/covariance_estimation.cpp) # src/edge_labeler.cpp)
#Detector Threshold Adapter
set(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/feature_adjuster.cpp src/aorb.cpp src/features.cpp)

if (${USE_SIFT_GPU})
 	set(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/sift_gpu_wrapper.cpp)
endif (${USE_SIFT_GPU})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS)



catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS ${catkin_RUN_PACKAGES}
  DEPENDS Eigen OpenCV G2O glut libglew-dev SuiteSparse
)

##############################################################################
# Binaries
##############################################################################
add_executable(rgbdslam 
  ${QT_SOURCES} 
  ${QT_RESOURCES_CPP} 
  ${QT_FORMS_HPP}  
  ${ADDITIONAL_SOURCES}
  ${QT_MOC_HPP} 
)
add_dependencies(rgbdslam rgbdslam_gencpp)


#set libs
#SET(G2O_LIBS cholmod g2o_core g2o_stuff g2o_types_slam3d g2o_solver_cholmod g2o_solver_pcg g2o_solver_csparse cxsparse g2o_incremental)
#SET(G2O_LIBS ${G2O_LIBRARIES} cholmod cxsparse)
#SET(LIBS_LINK GL GLU -lgl2ps ${G2O_LIBS} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS})
#Use specific version of cxsparse, to avoid linker warning about possible conflicts between versions 2.2.3 (linked from g2o) and 3.1.0
#SET(LIBS_LINK GL GLU cholmod /usr/lib/libcxsparse.so.2.2.3 ${G2O_LIBRARIES} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS})
SET(LIBS_LINK GL GLU cholmod cxsparse ${G2O_LIBRARIES} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS})

#-lboost_signals -lrt -loctomap -loctomap_server -loctomath)

#link libraries
target_link_libraries(rgbdslam opencv_nonfree ${LIBS_LINK}  ${catkin_LIBRARIES}  )

###############################################################################
## Bag Viewer
###############################################################################
#
#file(GLOB QT_FORMS2 RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES2 RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC2 RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/qtros.h src/bagloader.h src/qt_gui.h src/glviewer.h)
#
#QT4_ADD_RESOURCES(QT_RESOURCES_CPP2 ${QT_RESOURCES2})
#QT4_WRAP_UI(QT_FORMS_HPP2 ${QT_FORMS2})
#QT4_WRAP_CPP(QT_MOC_HPP2 ${QT_MOC2})  
#
#
#SET(ADDITIONAL_SOURCES2 src/result-bag-viewer.cpp src/bagloader.cpp src/qtros.cpp  src/qt_gui.cpp src/glviewer.cpp src/parameter_server.cpp src/misc.cpp src/ColorOctomapServer.cpp src/scoped_timer.cpp)
#add_executable(bagviewer ${QT_SOURCES2} ${QT_RESOURCES_CPP2} ${QT_FORMS_HPP2} ${QT_MOC_HPP2} ${ADDITIONAL_SOURCES2})
#target_link_libraries(bagviewer ${LIBS_LINK})
# 
