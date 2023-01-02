cmake_minimum_required(VERSION 3.10)

# set the project name
project(Camera)

# setup opencv
find_package( OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

option(USE_SHOW_IMAGE "Show image" OFF)
option(USE_SHOW_3D "Show 3D view (only for solvePnP)" OFF)
option(USE_SHOW_CALC "Show calculations" OFF)
option(USE_SHOW_TIME "Show timings" OFF)

if (USE_SHOW_IMAGE) 
	add_compile_definitions(SHOW_IMAGE)
endif (USE_SHOW_IMAGE) 

if (USE_SHOW_3D) 
	add_compile_definitions(SHOW_3D)
endif (USE_SHOW_3D)

if (USE_SHOW_CALC) 
	add_compile_definitions(SHOW_CALC)
endif (USE_SHOW_CALC)

if (USE_SHOW_TIME) 
	add_compile_definitions(SHOW_TIME)
endif (USE_SHOW_TIME)

include_directories( ${OpenCV_INCLUDE_DIR} ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")



# add the executable
add_executable( devastar compute.cpp main.cpp display3d.cpp aim_calib.cpp config.cpp ir_camera.cpp df_robot.cpp )
add_executable( test_config test_config.cpp config.cpp )
add_executable( test_calib test_calib.cpp aim_calib.cpp )

# set some flags
target_compile_features( devastar PRIVATE cxx_std_17)
target_compile_features( test_config PRIVATE cxx_std_17)
target_compile_features( test_calib PRIVATE cxx_std_17)

target_link_libraries( devastar ${OpenCV_LIBS}  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} gpiod )
target_link_libraries( test_config ${OpenCV_LIBS} )
target_link_libraries( test_calib ${OpenCV_LIBS} ) 
