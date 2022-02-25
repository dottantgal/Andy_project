#include "include/andy_ros1_pid_core.h"

AndyRos1PID::AndyRos1PID()
{
}

AndyRos1PID::~AndyRos1PID()
{
}

void AndyRos1PID::publishMessage(ros::Publisher *pub_message)
{
  andy_ros1_pid::PID msg;
  msg.p = p_;
  msg.d = d_;
  msg.i = i_;
  pub_message->publish(msg);
}

void AndyRos1PID::messageCallback(const andy_ros1_pid::PID::ConstPtr &msg)
{
  p_ = msg->p;
  d_ = msg->d;
  i_ = msg->i;

  //echo P,I,D
  ROS_INFO("P: %f", p_);
  ROS_INFO("D: %f", d_);
  ROS_INFO("I: %f", i_);
}

void AndyRos1PID::configCallback(andy_ros1_pid::andy_ros1_pidConfig &config, double level)
{
  //for PID GUI
  p_ = config.p;
  d_ = config.d;
  i_ = config.i;

}

