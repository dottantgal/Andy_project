 /**
 *  @file       motors_odom.h
 *  @brief      High level node to create motors odometry from encoders ticks
 *              This version is built up with classes.
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *
 */


#ifndef MOTORS_ODOM_H
#define MOTORS_ODOM_H


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>



class MotorsOdom
{
private:
  ros::NodeHandle nh_odom_;
  ros::Subscriber actual_vel_sub_;
  ros::Publisher odom_pub_;


  // Parameters
  int motors_odom_loop_rate_;
  bool publish_tf_;
  bool debug_;

  std::string NODE_ID_ = "[motors odom]";
  double vel_x_ = 0.0;
  double vel_y_ = 0.0;
  double vel_z_ = 0.0;

  void actualVelCallback(const geometry_msgs::Vector3Stamped& vel);
  void createOdometry();
  int getParams();
  int setLoggerLevel(bool debug);

protected:

public:
  MotorsOdom();
  ~MotorsOdom();

};

#endif  // MOTORS_ODOM_H
