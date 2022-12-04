cmake_minimum_required(VERSION 3.10)

# set the project name
project(Camera)

# setup opencv
find_package( OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIR} ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

# add the executable
add_executable( camera camera.cpp display3d.cpp aim_calib.cpp config.cpp )
add_executable( test_config test_config.cpp config.cpp )
add_executable( test_calib test_calib.cpp aim_calib.cpp )

# set some flags
target_compile_features( camera PRIVATE cxx_std_17)
target_compile_features( test_config PRIVATE cxx_std_17)
target_compile_features( test_calib PRIVATE cxx_std_17)

target_link_libraries( camera ${OpenCV_LIBS}  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} gpiod )
target_link_libraries( test_config ${OpenCV_LIBS} )
target_link_libraries( test_calib ${OpenCV_LIBS} ) 
