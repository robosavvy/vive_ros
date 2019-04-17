cmake_minimum_required(VERSION 2.8.3)
project(vive_ros)

## set definition, path, compileflags
add_definitions(-DLINUX -DLINUX64 -DPOSIX)
set(OPENVR "$ENV{HOME}/libraries/openvr")
add_definitions(-DOPENVR_PATH="${OPENVR}") ## for source code to get sample textures and shader files path
set(THIRDPARTY_DIR ${OPENVR}/samples/thirdparty)
set(SHARED_SRC_DIR ${OPENVR}/samples/shared)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -include ${SHARED_SRC_DIR}/compat.h")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

## find openvr
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
set(OPENVR_INCLUDE_DIR ${OPENVR}/headers ${OPENVR}/samples)

## find SDL
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

## common src from openvr sample
file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)

## find opengl (OpenGL specific)
find_package(OpenGL REQUIRED)
find_package(GLEW 1.11 REQUIRED)

## find vulkan (vulkan specific)
find_library(VULKAN_LIBRARY
  NAMES
    vulkan-1
    vulkan
  PATHS
    ${THIRDPARTY_DIR}/vulkan-1.0.49.0/lib
  PATH_SUFFIXES
    linux64
  NO_DEFAULT_PATH
)
set(VULKAN_INCLUDE_DIR ${THIRDPARTY_DIR}/vulkan-1.0.49.0/include)

## add include path
include_directories(
  ${SDL2_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR} ## (OpenGL specific)
  ${GLEW_INCLUDE_DIR} ## (OpenGL specific)
  ${VULKAN_INCLUDE_DIR} ## (Vulkan specific)
)

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

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# VIVE node:
add_executable(vive_node
  ${SHARED_SRC_FILES}
  src/vive_node.cpp
  src/vr_interface.cpp
)

## link all
target_link_libraries(vive_node
  ${catkin_LIBRARIES}
  ${OPENGL_LIBRARIES} ## (OpenGL specific)
  ${GLEW_LIBRARIES} ## (OpenGL specific)
  ${VULKAN_LIBRARY} ## (Vulkan specific)
  ${SDL2_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
)
