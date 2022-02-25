cmake_minimum_required(VERSION 3.0.2)
project(andy_ros1_pid)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  message_runtime
  roscpp
  std_msgs
)

add_message_files(
  FILES
  PID.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/andy_ros1_pid.cfg
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS dynamic_reconfigure message_runtime message_generation roscpp std_msgs
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

# Set the name to use for the executable.
set (BINNAME1 andy_ros1_pid_configure)
set (BINNAME2 andy_ros1_pid_listener)

# Set the source files to use with the executable.
set (SRCS1 ${SRCS1} src/andy_ros1_pid_core.cpp)
set (SRCS1 ${SRCS1} src/andy_ros1_pid_configure.cpp)
set (SRCS2 ${SRCS2} src/andy_ros1_pid_core.cpp)
set (SRCS2 ${SRCS2} src/andy_ros1_pid_listener.cpp)

# Build the executable that will be used to run this node.
add_executable (${BINNAME1} ${SRCS1})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})
add_dependencies(${BINNAME1} ${PROJECT_NAME}_gencfg)
add_executable (${BINNAME2} ${SRCS2})
target_link_libraries(${BINNAME2} ${catkin_LIBRARIES})
add_dependencies(${BINNAME2} ${PROJECT_NAME}_gencfg)
