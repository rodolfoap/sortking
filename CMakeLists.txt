cmake_minimum_required(VERSION 3.10)
project(sort-tracking)

# Force C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Equivalent to (CFLAGS) -g
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Info
message(STATUS "Binary app will go to: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "CMakeLists.txt contents:")
execute_process(COMMAND sed "s/^/\t/" CMakeLists.txt
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	RESULT_VARIABLE CMD_ERROR)

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "Include dirs ${OpenCV_INCLUDE_DIRS}")
message(STATUS "LINK LIBRARIES ${OpenCV_LIBS}")
include_directories(${OpenCV_INCLUDE_DIRS})

# Build library
add_library(sortcpp SHARED Hungarian.h Hungarian.cpp KalmanTracker.h KalmanTracker.cpp OnlineSortTracker.h OnlineSortTracker.cpp)
target_link_libraries(sortcpp ${OpenCV_LIBS})

# Build offline
add_executable(offline offline.cpp Hungarian.cpp KalmanTracker.cpp)
target_link_libraries(offline sortcpp)

# Build online
add_executable(online online.cpp)
target_link_libraries(online sortcpp)
