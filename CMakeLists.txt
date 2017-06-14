cmake_minimum_required(VERSION 2.8.3)
project(vive_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_srvs
  geometry_msgs
  cv_bridge
  image_transport
)

set(CMAKE_BUILD_TYPE "Release")

# TODO: make find script, to use find_package(openvr REQUIRED)
set(OPENVR "$ENV{HOME}/libraries/openvr")
find_library(OPENVR_LIB openvr_api "${OPENVR}/lib/linux64" NO_DEFAULT_PATH)
include_directories("${OPENVR}/headers" "${OPENVR}/samples")
set(OPENVR_SAMPLE_UTILS
  ${OPENVR}/samples/shared/pathtools.cpp
  ${OPENVR}/samples/shared/lodepng.cpp
  ${OPENVR}/samples/shared/Matrices.cpp
)
message("Found libopenvr_api: ${OPENVR_LIB}")
message(STATUS "OPENVR '${OPENVR}' ")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${OPENVR}/cmake")
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})
message(STATUS "SDL2_INCLUDE_DIR '${SDL2_INCLUDE_DIR}' ")

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
	
find_package(GTK2)
add_definitions(-DHAVE_GTK)
include_directories(${GTK2_INCLUDE_DIRS})

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})


###################################
## catkin specific configuration ##
###################################
#catkin_package()
catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES #${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge#roscpp
  DEPENDS #cv_bridge
  )

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
  ${OPENVR_SAMPLE_UTILS}
)
target_link_libraries(vive_node
  ${catkin_LIBRARIES}
  ${OPENVR_LIB}
  openvr_api
  ${GLEW_LIBRARIES}
  ${GTK2_LIBRARIES}
  ${SDL2_LIBRARY}
  ${OpenCV_LIBS}
#  dl
  GL
#  GLU
)

