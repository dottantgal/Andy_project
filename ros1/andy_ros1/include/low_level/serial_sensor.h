 /**
 *  @file       serial_sensor.h
 *  @brief      Declarations of variables, library links for serial communication nodes
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *  @author     Silvano Sallese <silvano.sallese@gmail.com>
 *
 *  @date       October 2017
 *  @copyright  Copyright (c) 2017 Hyperlync Technologies Inc, Orange Park, Florida U.S.A.
 *
 *              The Programs/Files (which include both the software and documentation)
 *              contain proprietary information of Hyperlync Technologies Inc;
 *              they are provided under a license agreement containing restrictions on use
 *              and disclosure and are also protected by copyright, patent, and other
 *              intellectual and industrial property laws.
 *
 *              Reverse engineering, disassembly, or decompilation of the Programs and Files
 *              is prohibited.
 */

#ifndef HYPROS_SERIALSENSOR
  #define HYPROS_SERIALSENSOR 1
#endif


//C++ include
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>  //Serial_sensor package include

#include <andy_ros1/pidConfig.h>
#include <andy_ros1/Ticks.h>
#include <hyperlync_ros/Ticks.h>
//-


using namespace std;

std::string NODE_ID_A = "[serial_pid_node] ";
std::string NODE_ID_B = "[serial_sensor_node] ";


/* ** Serial PID 3 parameters ** */

double Kp;
double Ki;
double Kd;
double Wheel_diameter;
double Base_width;
int Ticks_per_rev;
int out_min;
int out_max;
int max_rpm;
int serial_pid_loop_rate;
bool imu_message;
bool DEBUG_PID;
bool enable_low_level;
double rotation_ratio;

long left_ticks_enc = 0;
long right_ticks_enc = 0;

double required_linear_x_vel = 0.0;
double required_linear_y_vel = 0.0;
double required_angular_vel = 0.0;

double linear_vel_x_mins = 0.0;
double linear_vel_y_mins = 0.0;
double angular_vel_mins = 0.0;

double vel_target_right_rad_sec = 0.0;
double vel_target_left_rad_sec = 0.0;
double vel_target_right_minute = 0.0;
double vel_target_left_minute = 0.0;
double vel_target_right_rpm = 0.0;
double vel_target_left_rpm = 0.0;
double vel_target_right_linear = 0.0;
std_msgs::Float32 vel_right_linear;
double vel_target_left_linear = 0.0;
std_msgs::Float32 vel_left_linear;
double right_motor_vel_rpm = 0.0;
double left_motor_vel_rpm = 0.0;
double right_vel_msg = 0.0;
double left_vel_msg = 0.0;

ros::Time current_time;
double delta_time;
ros::Time previous_time(0.0);

double left_ticks_vel = 0.0;
double right_ticks_vel = 0.0;

double required_rpm_left = 0.0;
double required_rpm_right = 0.0;
double last_input_right = 0.0;
double last_input_left = 0.0;

double integral_left = 0.0;
double integral_right = 0.0;
double derivative_left = 0.0;
double derivative_right = 0.0;
double previous_error_left = 0.0;
double previous_error_right = 0.0;
double error_left = 0.0;
double pwm_left = 0.0;
double error_right = 0.0;
double pwm_right = 0.0;

std_msgs::Int16 motor_left_pwm;
std_msgs::Int16 motor_right_pwm;

long left_ticks_prev_encoder = 0;
long right_ticks_prev_encoder = 0;

geometry_msgs::Vector3Stamped raw_vel_msg;

/* *** */



/* ** Serial sensor PID parameters ** */

double vel_x = 0.0;
double vel_y = 0.0;
double vel_th = 0.0;
double vel_z = 0.0;

ros::Time last_vel_time(0.0);
ros::Time last_imu_time(0.0);
ros::Time last_loop_time(0.0);

int serial_sensor_pid_loop_rate;
bool imu_message_serial_sensor;
bool publish_tf;

double x = 0.0;
double y = 0.0;

double vel_dt = 0.0;
double imu_dt = 0.0;
double imu_z = 0.0;

/* *** */
