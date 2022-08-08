cmake_minimum_required(VERSION 3.10)

# set the project name
project(Devestar)

# setup output 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Add subdirectories
add_subdirectory(calib/calibrate)
add_subdirectory(calib/generate_board)
add_subdirectory(gun_camera)


