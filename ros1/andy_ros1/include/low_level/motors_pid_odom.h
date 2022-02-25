 /**
 *  @file       motors_pid_odom.h
 *  @brief      High level node to apply the PID control to the velocity of the motor and publish
 *              the wheels Odometry.
 *              This version is built up with classes.
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *
 */


#ifndef MOTORS_PID_ODOM_H
#define MOTORS_PID_ODOM_H


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <andy_ros1/pidConfig.h>
#include <andy_ros1/Ticks.h>
#include <andy_ros1/ResetTicks.h>


class MotorsPidOdom
{
private:
  ros::NodeHandle nh_pid_odom_;
  ros::ServiceClient reset_ticks_client_;
  ros::Subscriber vel_target_sub_;
  ros::Subscriber left_ticks_sub_;
  ros::Subscriber right_ticks_sub_;
  ros::Publisher pwm_left_pub_;
  ros::Publisher pwm_right_pub_;
  ros::Publisher actual_vel_pub_;
  ros::Publisher odom_pub_;
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
  int motors_pid_odom_loop_rate_;
  bool debug_;
  bool publish_tf_;
  // -

  // Dynamic reconfigure callbacks
  dynamic_reconfigure::Server<andy_ros1::pidConfig> server;
  dynamic_reconfigure::Server<andy_ros1::pidConfig>::CallbackType f;

  std::string NODE_ID_ = "[motors pid odom]";
  double delta_time_ = 0.0;
  double theta_pid_ = 0.0;
  double x_pos_ = 0.0;
  double y_pos_ = 0.0;
  int countdown_timer_ = 5;
  tf2::Quaternion odom_quat_;
  geometry_msgs::TransformStamped odom_trans_;
  tf2_ros::StaticTransformBroadcaster tf_br_;
  nav_msgs::Odometry odom_;
  andy_ros1::ResetTicks srv_;

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
  void reconfigureCallback(andy_ros1::pidConfig &config, uint32_t level);
  void performPidOdom();
  int constrainFunction(const double &x, const int &a, const int &b);
  int setLoggerLevel(bool debug);
  int getParams();

protected:

public:
  MotorsPidOdom();
  ~MotorsPidOdom();

};

#endif  // MOTORS_PID_ODOM_H
