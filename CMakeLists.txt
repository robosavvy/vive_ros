cmake_minimum_required(VERSION 2.8.3)
project(vive_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_srvs
  geometry_msgs
  openvr 
)

set(CMAKE_BUILD_TYPE "Release")

###################################
## catkin specific configuration ##
###################################
catkin_package()
catkin_add_env_hooks(openvr SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# TODO: find openvr
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# VIVE node:
add_executable(vive_node
  src/vive_node.cpp
  src/vr_interface.cpp
)
target_link_libraries(vive_node
  ${catkin_LIBRARIES}
)

