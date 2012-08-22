PROJECT(feature_tracker)

cmake_minimum_required(VERSION 2.4.6)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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

rosbuild_find_ros_package( pcl_ros )
rosbuild_find_ros_package( iap_common )

include_directories(include)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable(feature_tracker ${PROJECT_SOURCE_DIR}/src/FeatureTracker.cpp )
target_link_libraries(feature_tracker ${iap_common_LIBRARIES})