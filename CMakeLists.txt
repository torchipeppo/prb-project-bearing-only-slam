cmake_minimum_required(VERSION 2.8)

# init project and compiler flags
project(bearingslam)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

# add dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# include main executables
add_subdirectory(executables)

# include test builds
add_subdirectory(tests)
