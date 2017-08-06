cmake_minimum_required(VERSION 2.8.3)
project(vive_ros)

set(OPENVR "$ENV{HOME}/openvr")

## import from openvr sample
add_definitions(-DLINUX -DLINUX64 -DPOSIX)
set(THIRDPARTY_DIR ${OPENVR}/samples/thirdparty)
set(SHARED_SRC_DIR ${OPENVR}/samples/shared)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -include ${SHARED_SRC_DIR}/compat.h")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
find_package(OpenGL REQUIRED)
find_package(GLEW 1.11 REQUIRED)

set(SDL_REQUIRED_LIBRARIES SDL2)
foreach(lib ${SDL_REQUIRED_LIBRARIES})
  find_library(${lib}_LIBRARY
    NAMES
      ${lib}
    PATHS
      ${THIRDPARTY_DIR}/sdl2-2.0.3/bin
    PATH_SUFFIXES
      linux64
    )
  list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
endforeach()
set(SDL2_INCLUDE_DIR ${THIRDPARTY_DIR}/sdl2-2.0.3/include)

find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${OPENVR}/bin
    ${OPENVR}/lib
  PATH_SUFFIXES
    linux64
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${OPENVR}/headers)

file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)

include_directories(
  ${OPENGL_INCLUDE_DIR}
  ${GLEW_INCLUDE_DIR}
  ${SDL2_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
)

# add_executable(${TARGET_NAME}
#   ${SHARED_SRC_FILES}
#   hellovr_opengl_main.cpp
# )

# target_link_libraries(${TARGET_NAME} 
#   ${OPENGL_LIBRARIES}
#   ${GLEW_LIBRARIES}
#   ${SDL2_LIBRARIES}
#   ${OPENVR_LIBRARIES}
#   ${CMAKE_DL_LIBS}
#   ${EXTRA_LIBS}
# )


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

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${OPENVR}/headers
)

# VIVE node:
add_executable(vive_node
  ${SHARED_SRC_FILES}
  # hellovr_opengl_main.h
  src/vive_node.cpp
  src/vr_interface.cpp
)

target_link_libraries(vive_node
  ${catkin_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${SDL2_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
)