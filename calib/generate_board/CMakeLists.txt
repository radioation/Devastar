cmake_minimum_required(VERSION 3.10)

# set the project name
project(GenerateBoard)

# setup opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIR} )

# add the executable
add_executable( generate_board generate_board.cpp)

target_link_libraries( generate_board ${OpenCV_LIBS} )
