cmake_minimum_required(VERSION 3.10)

# set the project name
project(CalibrateCamera)

# setup opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIR} )

# add the executable
add_executable( calibrate calibrate.cpp)

target_link_libraries( calibrate ${OpenCV_LIBS} )
