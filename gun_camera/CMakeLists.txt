cmake_minimum_required(VERSION 3.10)

# set the project name
project(Camera)

# setup opencv
find_package( OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIR} ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

# add the executable
add_executable( camera camera.cpp display3d.cpp)

target_link_libraries( camera ${OpenCV_LIBS}  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} gpiod )
