cmake_minimum_required(VERSION 3.0.2)
project(fingertips)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
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

include_directories(/opt/ros/noetic/include /opt/ros/noetic/lib)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                
	${PROJECT_SOURCE_DIR}/bin)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
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
  src/fingertip_contact_pub.py
  src/VisualFeedback.py
  src/egg_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

