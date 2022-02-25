 /**
 *  @file       motors_pid.h
 *  @brief      High level node to apply the PID control to the velocity of the motor and publish
 *              the PWM directly for the motor driver.
 *              This version is built up with classes.
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *
 */


#ifndef MOTORS_PID_H
#define MOTORS_PID_H


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include <andy_ros1/pidConfig.h>
#include <andy_ros1/Ticks.h>


class MotorsPid
{
private:
  ros::NodeHandle nh_pid_;
  ros::Subscriber vel_target_sub_;
  ros::Subscriber left_ticks_sub_;
  ros::Subscriber right_ticks_sub_;
  ros::Publisher pwm_left_pub_;
  ros::Publisher pwm_right_pub_;
  ros::Publisher actual_vel_pub_;
  ros::Time current_time_;
  ros::Time previous_time_;
  geometry_msgs::Vector3Stamped raw_vel_msg_;


  // Dinamic reconfigure parameters
  double kp_;
  double ki_;
  double kd_;
  double wheel_diameter_;
  double base_width_;
  int ticks_per_rev_;
  int out_min_;
  int out_max_;
  int max_rpm_;
  int motors_pid_loop_rate_;
  bool debug_;
  // -

  // Dynamic reconfigure callbacks
  dynamic_reconfigure::Server<andy_ros1::pidConfig> server;
  dynamic_reconfigure::Server<andy_ros1::pidConfig>::CallbackType f;

  std::string NODE_ID_ = "[motors pid]";
  double delta_time_ = 0.0;

  // left motor vars
  double left_motor_ticks_ = 0.0;
  double previous_left_motor_ticks_ = 0.0;
  double vel_target_left_motor_rpm_ = 0.0;
  double left_motor_vel_rpm_ = 0.0;
  double error_left_ = 0.0;
  double previous_error_left_ = 0.0;
  double integral_left_ = 0.0;
  double derivative_left_ = 0.0;
  double pwm_left_ = 0.0;
  std_msgs::Int16 motor_left_pwm_;


  // right motor vars
  double right_motor_ticks_ = 0.0;
  double previous_right_motor_ticks_ = 0.0;
  double vel_target_right_motor_rpm_ = 0.0;
  double right_motor_vel_rpm_ = 0.0;
  double error_right_ = 0.0;
  double previous_error_right_ = 0.0;
  double integral_right_ = 0.0;
  double derivative_right_ = 0.0;
  double pwm_right_ = 0.0;
  std_msgs::Int16 motor_right_pwm_;


  void leftTicksCallback(const andy_ros1::Ticks& left_msg);
  void rightTicksCallback(const andy_ros1::Ticks& right_msg);
  void velTargetCallback(const geometry_msgs::Twist& msg);
  void performPid();
  int constrainFunction(const double &x, const int &a, const int &b);
  void reconfigureCallback(andy_ros1::pidConfig &config, uint32_t level);
  int setLoggerLevel(bool debug);
  int getParams();

protected:

public:
  MotorsPid();
  ~MotorsPid();

};

#endif  // MOTORS_PID_H
