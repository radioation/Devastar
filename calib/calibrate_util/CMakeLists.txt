cmake_minimum_required(VERSION 3.10)

# set the project name
project(CalibrateUtil)

# setup dependencies
find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIR} ${Boost_INCLUDE_DIR} )
include_directories( ${OpenCV_INCLUDE_DIR} )
find_package( Boost COMPONENTS program_options REQUIRED )

# add the executable
add_executable( calibrate_util calibrate_util.cpp)

target_link_libraries( calibrate_util ${OpenCV_LIBS} Boost::program_options )
