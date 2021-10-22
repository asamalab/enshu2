cmake_minimum_required(VERSION 3.0.2)
project(enshu2)

find_package(OpenCV REQUIRED)

include_directories(
${OpenCV_INCLUDE_DIRS} include
)
add_executable(
 0_rgb_process src/rgb_process.cpp
)
target_link_libraries(
 0_rgb_process
 ${OpenCV_LIBRARIES}
)
add_executable(
 1_hsv_process src/hsv_process.cpp
)
target_link_libraries(
 1_hsv_process
 ${OpenCV_LIBRARIES}
)
