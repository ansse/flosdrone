cmake_minimum_required(VERSION 2.8.3)
project(flosdrone)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flosdrone
#  CATKIN_DEPENDS ardrone_autonomy cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(flosdrone_node
  src/flosdrone_node.cpp
  src/flosdrone_controller.cpp
  src/flosdrone_controller.h
  src/flosdrone_estimator.cpp
  src/flosdrone_estimator.h
  src/flosdrone_ui.cpp
  src/flosdrone_ui.h
)


## Specify libraries to link a library or executable target against
target_link_libraries(flosdrone_node
  ${catkin_LIBRARIES}
  ${GLFW_STATIC_LIBRARIES}
  GLU
)
