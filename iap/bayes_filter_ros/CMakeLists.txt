cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


#Source the files from Bayes++ and compile the library
SET(CMAKE_CXX_FLAGS "-ftemplate-depth-128 -finline-functions -Wno-inline -Wall")
rosbuild_add_boost_directories()
#add_library(BayesFilter STATIC src/BayesFilter/bayesFlt.cpp
rosbuild_add_library(BayesFilter src/BayesFilter/bayesFlt.cpp
                src/BayesFilter/bayesFltAlg.cpp
		src/BayesFilter/CIFlt.cpp
		src/BayesFilter/covFlt.cpp
		src/BayesFilter/infFlt.cpp
		src/BayesFilter/infRtFlt.cpp
		src/BayesFilter/itrFlt.cpp
		src/BayesFilter/matSup.cpp
		src/BayesFilter/SIRFlt.cpp
		src/BayesFilter/UDFlt.cpp
		src/BayesFilter/UdU.cpp
		src/BayesFilter/unsFlt.cpp
		src/BayesFilter/bayesFlt.cpp)
