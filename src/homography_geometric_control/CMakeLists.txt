cmake_minimum_required(VERSION 3.0.2)
project(homography_geometric_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs


  #自己写的库
  mavros_interaction
  uav_state
  so3_control
  #自己定义的消息格式
  homo_msgs
)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  #INCLUDE_DIRS include
#  LIBRARIES homography_geometric_control
  CATKIN_DEPENDS roscpp rospy std_msgs uav_state mavros_interaction so3_control
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/HomographyGeometricControl.cpp
)

add_executable(homo_geo test/homo_geo.cpp)
target_link_libraries(homo_geo 
${catkin_LIBRARIES}
${PROJECT_NAME}
)

add_executable(homography_publish test/homography_publish.cpp)
target_link_libraries(homography_publish 
${catkin_LIBRARIES}
${PROJECT_NAME}
)
