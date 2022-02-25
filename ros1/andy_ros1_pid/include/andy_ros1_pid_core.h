#ifndef ANDY_ROS1_PID_CORE_H
#define ANDY_ROS1_PID_CORE_H

#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include <andy_ros1_pid/PID.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <andy_ros1_pid/andy_ros1_pidConfig.h>

class AndyRos1PID
{
public:
  AndyRos1PID();
  ~AndyRos1PID();
  void configCallback(andy_ros1_pid::andy_ros1_pidConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);
  void messageCallback(const andy_ros1_pid::PID::ConstPtr &msg);

  double p_;
  double d_;
  double i_;

};
#endif
