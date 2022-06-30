cmake_minimum_required(VERSION 3.10)

# set the project name
project(Camera)

# setup opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIR} )

# add the executable
add_executable( camera camera.cpp)

target_link_libraries( camera ${OpenCV_LIBS} )
