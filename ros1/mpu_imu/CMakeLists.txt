cmake_minimum_required(VERSION 3.0.2)
project(mpu_imu)

## Compile as C++11, supported in ROS Kinetic and newer
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs std_srvs roslaunch tf2 tf2_ros  tf)
#roslaunch_add_file_check(launch)
## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs std_msgs std_srvs tf tf2 tf2_ros)#serial

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(imu_type_creator src/imu_type_creator.cpp)
target_link_libraries(imu_type_creator ${catkin_LIBRARIES})

#add_executable(imu_type_creator_uno src/imu_type_creator_uno.cpp)
#target_link_libraries(imu_type_creator_uno ${catkin_LIBRARIES})

add_executable(imu_type_creator_with_quat src/imu_type_creator_with_quat.cpp)
target_link_libraries(imu_type_creator_with_quat ${catkin_LIBRARIES})

add_executable(imu_type_creator_tf2_6500 src/imu_type_creator_tf2_6500.cpp)
target_link_libraries(imu_type_creator_tf2_6500 ${catkin_LIBRARIES})

add_executable(imu_type_creator_tf2_6050 src/imu_type_creator_tf2_6050.cpp)
target_link_libraries(imu_type_creator_tf2_6050 ${catkin_LIBRARIES})

add_executable(odom_creator src/odom_creator.cpp)
target_link_libraries(odom_creator ${catkin_LIBRARIES})
