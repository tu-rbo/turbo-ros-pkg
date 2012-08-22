PROJECT(kinematic_analyzer)

cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)

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
  	set(ROS_CURRENT_VERSION electric)
  elseif(${__version} STREQUAL "fuerte")
  	message("[rosbuild] Using ROS fuerte")
    add_definitions(-DROS_FUERTE)
    set(ROS_CURRENT_VERSION electric)
  else()
  	message(FATAL_ERROR "[rosbuild] Unknown version of ROS: "${__version})
  endif()
endif(_rosversion_failed OR NOT __version)

rosbuild_find_ros_package( iap_common )
rosbuild_find_ros_package( flann )

# Add eigen as an external library if we are compiling on ROS fuerte
if(${ROS_CURRENT_VERSION} STREQUAL "fuerte")
	find_package(Eigen REQUIRED)
	include_directories(${Eigen_INCLUDE_DIRS})
endif(${ROS_CURRENT_VERSION} STREQUAL "fuerte")

include_directories(include/)
include_directories(include/kinematic_analyzer)
include_directories(include/kinematic_analyzer/joint)
include_directories(include/kinematic_analyzer/joint_estimation)

include_directories(Bayes++)

add_definitions(-DEIGEN2_SUPPORT)

# generate messages
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Generates a list of all .cpp files in /src and subdirectories
FILE(GLOB source_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/joint/*.cpp"  
	"src/joint_estimation/*.cpp" 
	"src/joint_estimation/transformation_based/*.cpp" 
	"src/joint_estimation/probabilistic/*.cpp" 
	"src/joint_estimation/recursive/*.cpp"
	"src/joint_estimation/feature_based/*.cpp"
	"src/KinematicAnalyzer.cpp" 
	"src/utils.cpp" )
FILE(GLOB main_file RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/KinematicAnalyzerNode.cpp" )

# kinematic analyzer library (speeds up building)
rosbuild_add_library(ka ${source_files})

rosbuild_add_executable(kinematic_analyzer ${source_files} ${main_file})
target_link_libraries(kinematic_analyzer ${iap_common_LIBRARIES} BayesFilter)

## Tests ##
# FIXME Test
#rosbuild_add_gtest(kinematic_analyzer_test test/kinematic_analyzer_test.cpp)
#target_link_libraries(kinematic_analyzer_test ${PROJECT_NAME} ${OpenCV_LIBS} cv_bridge ${PCL_LIBS})

rosbuild_add_gtest(ros_messages_test test/ros_messages_test.cpp)
target_link_libraries(ros_messages_test ka ${OpenCV_LIBS} cv_bridge ${PCL_LIBS})
