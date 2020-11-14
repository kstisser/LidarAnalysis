cmake_minimum_required(VERSION 3.0)
project(post_process_pointcloud)
enable_language(CXX)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV ${OCV_VERSION} COMPONENTS core highgui imgproc REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PointCountAndColor.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES post_process_pointcloud
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge dynamic_reconfigure image_transport message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include_directories( SYSTEM
    ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp")

rosbuild_add_executable(${PROJECT_NAME}_node ${SOURCES})
set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${LINK_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)  

