cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Configuration
set(USE_SIFT_GPU 1)

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

# Shared library
rosbuild_add_library(shared src/math2d.cpp src/fps_calculator.cpp)

# Perception library
rosbuild_add_library(
  perception
  src/object_recognizer.cpp
  src/graham_scanner.cpp
  src/siftgpu_feature_detector.cpp
  src/siftgpu_descriptor_matcher.cpp
)

target_link_libraries(perception shared)

# Build SiftGPU + link with perception library
IF (${USE_SIFT_GPU})
  # Setup SiftGPU directories
	set(sift_gpu_build_path ${CMAKE_SOURCE_DIR}/external/SiftGPU/)
	include_directories(${CMAKE_CURRENT_BINARY_DIR} external/SiftGPU/src)
	# Compile SiftGPU
	message("\n------------------------------------------------------------------\n")
	message("Compiling SiftGPU...")
	execute_process(
		COMMAND make ${ROS_PARALLEL_JOBS} siftgpu 
		WORKING_DIRECTORY ${sift_gpu_build_path}
		RESULT_VARIABLE sift_gpu_return
	)
	message("\n------------------------------------------------------------------\n")
	# Error
	IF (NOT ${sift_gpu_return} EQUAL 0)
		message(FATAL_ERROR "SiftGPU cannot be compiled. Returned: ${sift_gpu_return}")
	ENDIF (NOT ${sift_gpu_return} EQUAL 0)
	# Success: Copy compiled library to lib folder
	file(
	  COPY external/SiftGPU/bin/libsiftgpu.so
	  DESTINATION ../lib/
	)
	# Tell other sources to use SiftGPU
	add_definitions(-DUSE_SIFT_GPU)
	# Link with perception
  target_link_libraries(perception siftgpu)
ENDIF (${USE_SIFT_GPU})

# Node database
rosbuild_add_executable(init_database src/init_database.cpp src/image_loader.cpp src/database.cpp)

# Node recognition
rosbuild_add_executable(init_recognition src/init_recognition.cpp)
target_link_libraries(init_recognition shared perception)

# Node semantic mapping
rosbuild_add_executable(init_mapping src/init_mapping.cpp src/semantic_map.cpp)

# Node visualization
rosbuild_add_executable(init_example src/init_example.cpp)

# Link with boost
target_link_libraries(init_database boost_filesystem boost_system)
target_link_libraries(init_recognition boost_signals)
