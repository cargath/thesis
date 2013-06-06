cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type. Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# Set the default path for built executables to the "bin" directory.
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory.
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Uncomment if you have defined messages.
rosbuild_genmsg()
# Uncomment if you have defined services.
rosbuild_gensrv()

# Math library
rosbuild_add_library(lib_math src/math2d.cpp)

# Object recognition library
rosbuild_add_library(lib_recognition src/object_recognizer.cpp src/dbscan.cpp src/graham_scanner.cpp)

# Node webcam image publisher
rosbuild_add_executable(init_webcam src/init_webcam.cpp)

# Node database
rosbuild_add_executable(init_database src/init_database.cpp src/image_loader.cpp)

# Node recognition
rosbuild_add_executable(init_recognition src/init_recognition.cpp)
target_link_libraries(init_recognition lib_math lib_recognition)

# Node semantic mapping
rosbuild_add_executable(init_mapping src/init_mapping.cpp src/semantic_map.cpp)

# Link with boost
target_link_libraries(init_database boost_filesystem boost_system)
target_link_libraries(lib_recognition boost_filesystem boost_system)
target_link_libraries(init_recognition boost_signals)
