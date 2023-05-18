cmake_minimum_required(VERSION 3.0.2)
project(polaris)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
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

include_directories(/opt/ros/noetic/include /opt/ros/noetic/lib)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                
	${PROJECT_SOURCE_DIR}/bin)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  #tf2_ros
  #tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)


## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  sksurgerynditracker/polaris_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )



