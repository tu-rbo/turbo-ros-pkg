PROJECT(visual_net)

cmake_minimum_required(VERSION 2.4.6)

#### DEBUG INFO
SET(CMAKE_BUILD_TYPE debug)
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
SET(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
####

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

rosbuild_init()

execute_process(
  COMMAND rosversion -d
  ERROR_VARIABLE __rosversion_err_ignore
  OUTPUT_VARIABLE __version
  RESULT_VARIABLE _rosversion_failed
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_rosversion_failed OR NOT __version)
  message(FATAL_ERROR "[rosbuild] Failed to find version of ROS")
else(_rosversion_failed OR NOT __version)
  if(${__version} STREQUAL "electric")
  	message("[rosbuild] Using ROS electric")
  	add_definitions(-DROS_ELECTRIC)
  elseif(${__version} STREQUAL "fuerte")
  	message("[rosbuild] Using ROS fuerte")
    add_definitions(-DROS_FUERTE)
  else()
  	message(FATAL_ERROR "[rosbuild] Unknown version of ROS: "${__version})
  endif()
endif(_rosversion_failed OR NOT __version)

rosbuild_find_ros_package( iap_common )
rosbuild_find_ros_package( flann )



include_directories(include/visual_net)
include_directories(include/visual_net/graph)
include_directories(include/visual_net/predictors)
include_directories(include/visual_net/sfm/bundler)
include_directories(include/visual_net/triangulation)
include_directories(include/visual_net/vision/camera)
include_directories(include/visual_net/vision/segment)
include_directories(include/visual_net/sfm/sfm)

include_directories(libs/lib/5point)
include_directories(libs/lib/ann_1.1_char/include)
include_directories(libs/lib/cblas)
include_directories(libs/lib/imagelib)
include_directories(libs/matrix)
include_directories(libs/lib/sba-1.5)
include_directories(libs/lib/sfm-driver)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Generates a list of all .cpp files in /src and subdirectories
FILE(GLOB source_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/visual_net/*.cpp" "src/visual_net/graph/*.cpp" "src/visual_net/predictors/*.cpp" "src/visual_net/sfm/bundler/*.cpp" "src/visual_net/sfm/sfm/*.cpp" "src/visual_net/triangulation/*.cpp" "src/visual_net/vision/camera/*.cpp" )

LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/libs)
#LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/libs/libs/ann_1.1_char/lib)

rosbuild_add_executable(visual_net ${source_files})
target_link_libraries(visual_net ${iap_common_LIBRARIES} image sfmdrv sba.v1.5 matrix z lapack blas cblas minpack m 5point jpeg ANN_char gfortran)
