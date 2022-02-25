cmake_minimum_required(VERSION 3.0.2)
project(andy_ros1)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_runtime
  sensor_msgs
  dynamic_reconfigure
  hyperlync_ros
  tf
  tf2
  tf2_ros
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Ticks.msg
  ActualVel.msg
)


add_service_files(
  FILES
  ResetTicks.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  config/pid.cfg
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime roscpp std_msgs sensor_msgs message_generation tf2 tf2_ros tf
)

###########
## Build ##
###########
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(serial_pid src/low_level/serial_pid.cpp)
target_link_libraries(serial_pid ${catkin_LIBRARIES})
add_dependencies(serial_pid ${catkin_EXPORTED_TARGETS})

add_executable(motors_pid src/low_level/motors_pid.cpp)
target_link_libraries(motors_pid ${catkin_LIBRARIES})

add_executable(motors_odom src/low_level/motors_odom.cpp)
target_link_libraries(motors_odom ${catkin_LIBRARIES})

add_executable(motors_pid_odom src/low_level/motors_pid_odom.cpp)
target_link_libraries(motors_pid_odom ${catkin_LIBRARIES})

#add_executable(serial_pid_andy src/low_level/serial_pid_andy.cpp)
#target_link_libraries(serial_pid_andy ${catkin_LIBRARIES})

add_executable(serial_sensor_pid src/low_level/serial_sensor_pid.cpp)
target_link_libraries(serial_sensor_pid ${catkin_LIBRARIES})
