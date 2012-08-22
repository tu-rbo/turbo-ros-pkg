PROJECT(iap_common)
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

INCLUDE_DIRECTORIES(include/iap_common)
INCLUDE_DIRECTORIES(include/iap_common/cluster)
INCLUDE_DIRECTORIES(include/iap_common/data_logging)
INCLUDE_DIRECTORIES(include/iap_common/image)
INCLUDE_DIRECTORIES(include/iap_common/kinematics)
INCLUDE_DIRECTORIES(include/iap_common/tracker)
INCLUDE_DIRECTORIES(include/iap_common/utils)
INCLUDE_DIRECTORIES(include/iap_common/video_sensor)
INCLUDE_DIRECTORIES(include/iap_common/cluster_generator)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

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
#rosbuild_find_ros_package( opencv2 )
rosbuild_find_ros_package( boost )

find_package(OpenCV 2.3 REQUIRED)

#set the default path for built executables to the "bin" directory
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# ----------------------------------------------------------------------------
#  Variables to set in iap_common_lib-config.cmake.in
# ----------------------------------------------------------------------------
set(PACKAGE "iap_common_lib")
set(PACKAGE_NAME "iap_common_lib")
set(CMAKE_INCLUDE_DIRS_CONFIGCMAKE "${PROJECT_SOURCE_DIR}/include/iap_common" 
	"${PROJECT_SOURCE_DIR}/include/iap_common/image" 
	"${PROJECT_SOURCE_DIR}/include/iap_common/kinematics" 
	"${PROJECT_SOURCE_DIR}/include/iap_common/tracker" 
	"${PROJECT_SOURCE_DIR}/include/iap_common/state_pattern" 
	"${PROJECT_SOURCE_DIR}/include/iap_common/utils" 
	"${PROJECT_SOURCE_DIR}/include/iap_common/video_sensor" 
	"${PROJECT_SOURCE_DIR}/include/iap_common/cluster" 
	"${PROJECT_SOURCE_DIR}/include/iap_common/cluster_generator" )
set(CMAKE_LIB_DIRS_CONFIGCMAKE "${PROJECT_SOURCE_DIR}/lib")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/iap_common_lib-config.cmake.in" "${CMAKE_BINARY_DIR}/iap_common_lib-config.cmake" IMMEDIATE @ONLY)
# ----------------------------------------------------------------------------

# Generates a list of all .cpp files in /src and subdirectories
FILE(GLOB source_files "src/*.cpp" "src/cluster/*.cpp" "src/image/*.cpp" "src/kinematics/*.cpp" "src/tracker/*.cpp" "src/utils/*.cpp" "src/video_sensor/*.cpp" "src/cluster_generator/*.cpp" )

rosbuild_add_library(${PROJECT_NAME} ${source_files})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} cv_bridge ${PCL_LIBS})
#rosbuild_add_compile_flags (kinect_tracking ${SSE_FLAGS})
#target_link_libraries(kinect_tracking cv_bridge)

## Tests ##
# TODO fix test
#rosbuild_add_gtest(feature_classes_test test/feature_classes_test.cpp)
#target_link_libraries(feature_classes_test ${PROJECT_NAME} ${OpenCV_LIBS} cv_bridge ${PCL_LIBS})

rosbuild_add_gtest(body_trajectory_test test/body_trajectory_test.cpp)
target_link_libraries(body_trajectory_test ${PROJECT_NAME} ${OPENCV_LIBRARIES} cv_bridge ${PCL_LIBS})

rosbuild_add_gtest(feature_classes_inout_test test/feature_classes_inout.cpp)
target_link_libraries(feature_classes_inout_test ${PROJECT_NAME} ${OPENCV_LIBRARIES} cv_bridge ${PCL_LIBS})

rosbuild_add_gtest(ros_messages_test test/ros_messages_test.cpp)
target_link_libraries(ros_messages_test ${PROJECT_NAME} ${OPENCV_LIBRARIES} cv_bridge ${PCL_LIBS})
