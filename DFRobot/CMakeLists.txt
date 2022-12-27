cmake_minimum_required(VERSION 3.10)

# set the project name
project(DFRobotTest)

# setup opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIR} )

# add the executable
add_executable( dfrobot_test test.cpp)

target_link_libraries( dfrobot_test ${OpenCV_LIBS} )
