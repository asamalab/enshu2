cmake_minimum_required(VERSION 3.0.2)
project(enshu2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  enshu_msgs
  turtlebot3_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   HandResult.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES enshu2
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
find_package(OpenCV REQUIRED)
include_directories(
 ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include
)
add_executable(
 0_rgb_process src/rgb_process.cpp
)
target_link_libraries(
 0_rgb_process
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)
add_executable(
 1_hsv_process src/hsv_process.cpp
)
target_link_libraries(
 1_hsv_process
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)

add_executable(
 2_zoom_green src/zoom_green.cpp
)
target_link_libraries(
 2_zoom_green
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)

add_executable(
 3_zoom_depth src/zoom_depth.cpp
)
target_link_libraries(
 3_zoom_depth
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)

add_executable(
  4_hand_gestures src/hand_gestures.cpp
)
target_link_libraries(
 4_hand_gestures
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)
add_dependencies(
  4_hand_gestures
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  5_hand_control src/hand_control.cpp
)
target_link_libraries(
  5_hand_control
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)
add_dependencies(
  5_hand_control
  ${catkin_EXPORTED_TARGETS}
)
