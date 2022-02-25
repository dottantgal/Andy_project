 /**
 *  @file       motors_pid_odom.cpp
 *  @brief      High level node to apply the PID control to the velocity of the motor and publish
 *              the wheels Odometry
 *              This version is built up with classes.
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *
 */


#include <include/low_level/motors_pid_odom.h>


MotorsPidOdom::MotorsPidOdom()
{
  ROS_INFO_STREAM(NODE_ID_ << "Starting motors PID ODOM node");

  f = boost::bind(&MotorsPidOdom::reconfigureCallback, this, _1, _2);
  server.setCallback(f);

  int res = getParams();

  if (res != 0)
  {
    ROS_ERROR_STREAM(NODE_ID_ << "Ops, it's not possible to get required parameters!");
  }

  reset_ticks_client_ = nh_pid_odom_.serviceClient<andy_ros1::ResetTicks>("reset_ticks_service");
  vel_target_sub_ = nh_pid_odom_.subscribe("/cmd_vel", 10, &MotorsPidOdom::velTargetCallback, this);
  left_ticks_sub_ = nh_pid_odom_.subscribe("/andy_ros1/left_motor_ticks", 1, &MotorsPidOdom::leftTicksCallback, this);
  right_ticks_sub_ = nh_pid_odom_.subscribe("/andy_ros1/right_motor_ticks", 1, &MotorsPidOdom::rightTicksCallback, this);
  pwm_left_pub_ = nh_pid_odom_.advertise<std_msgs::Int16>("/andy_ros1/left_motor_pwm",10);
  pwm_right_pub_ = nh_pid_odom_.advertise<std_msgs::Int16>("/andy_ros1/right_motor_pwm",10);
  actual_vel_pub_ = nh_pid_odom_.advertise<geometry_msgs::Vector3Stamped>("/actual_vel", 10);
  odom_pub_ = nh_pid_odom_.advertise<nav_msgs::Odometry>("/raw_odom", 10);

  // temporary solution to wait the low level connection
  //ros::Duration(5).sleep();
  ROS_INFO_STREAM(NODE_ID_ << " Resetting the motors ticks counts");
  while(countdown_timer_>0)
  {
      ROS_INFO_STREAM(NODE_ID_ << " " << countdown_timer_ << " remaining");
      ros::Duration(1).sleep();
      --countdown_timer_;
  }

  // these twos give the following error
  // """Service call failed: service [/reset_ticks_service] responded with an error:
  // error processing request: 'ServiceServer' object has no attribute 'id'"""
  //reset_ticks_client_.waitForExistence();
  //ros::service::waitForService("/reset_ticks_service");

  srv_.request.reset = true;
  if (reset_ticks_client_.call(srv_))
  {
    ROS_INFO_STREAM(NODE_ID_ << " Motor ticks reset successful");
  }
  else
  {
    ROS_ERROR_STREAM(NODE_ID_ << " Failed to reset ticks counts");
  }

  countdown_timer_ = 5;
  // wait to get 0 ticks
  ROS_INFO_STREAM(NODE_ID_ << " Starting the motors connection");
  while(countdown_timer_>0)
  {
      ROS_INFO_STREAM(NODE_ID_ << " " << countdown_timer_ << " remaining");
      ros::Duration(1).sleep();
      --countdown_timer_;
  }
  ROS_INFO_STREAM(NODE_ID_ << " Motors connection done");

  ros::Rate loop_rate(motors_pid_odom_loop_rate_);

  while(ros::ok())
  {
     performPidOdom();
     ros::spinOnce();
     loop_rate.sleep();
  }

}


MotorsPidOdom::~MotorsPidOdom() {}


void MotorsPidOdom::reconfigureCallback(andy_ros1::pidConfig &config, uint32_t level)
{
  ROS_DEBUG_STREAM(NODE_ID_ << "Reconfigure Request: \n"
    << "Kp gain                     : " << config.Kp << "\n"
    << "Ki gain                     : " << config.Ki << "\n"
    << "Kd gain                     : " << config.Kd << "\n");
  kp_ = config.Kp;
  ki_ = config.Ki;
  kd_ = config.Kd;
}


int MotorsPidOdom::constrainFunction(const double &x, const int &a, const int &b)
{
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}


int MotorsPidOdom::setLoggerLevel(bool debug)
{
  if (debug_) // try to set Debug level
  {
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();  // set Debug level
    }
    else if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
    {
      ros::console::notifyLoggerLevelsChanged();  // set Info level
      return 0;
    }
  }
  else  // set Info level
  {
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
    {
      ros::console::notifyLoggerLevelsChanged();  // set Info level
      return 0;
    }
  }

  return 1;
}


int MotorsPidOdom::getParams()
{
  int res = 0;

  // enable/disable debug lines
  ros::param::get("~debug", debug_);

  // set rosconsole level for logging
  res = setLoggerLevel(debug_);
  if (res)
    ROS_DEBUG_STREAM(NODE_ID_ << "Debug level enabled");

  res = 0;

  ros::param::get("~Kp", kp_);

  ros::param::get("~Kd", kd_);

  ros::param::get("~Ki", ki_);

  ros::param::get("~Wheel_diameter", wheel_diameter_);

  ros::param::get("~Base_width", base_width_);

  ros::param::get("~Ticks_per_rev", ticks_per_rev_);

  ros::param::get("~out_min", out_min_);

  ros::param::get("~out_max", out_max_);

  ros::param::get("~max_rpm", max_rpm_);

  ros::param::get("~publish_tf", publish_tf_);

  ros::param::get("~motors_pid_odom_loop_rate", motors_pid_odom_loop_rate_);

  return 0;
}


void MotorsPidOdom::leftTicksCallback(const andy_ros1::Ticks& left_msg)
{
  left_motor_ticks_ = left_msg.ticks;
}


void MotorsPidOdom::rightTicksCallback(const andy_ros1::Ticks& left_msg)
{
  right_motor_ticks_ = left_msg.ticks;
}


void MotorsPidOdom::velTargetCallback(const geometry_msgs::Twist& msg)
{

  float linear_vel_x_mins = 0.0;
  float linear_vel_y_mins = 0.0;
  float angular_vel_z_mins = 0.0;
  float tangential_vel = 0.0;
  float x_rpm = 0.0;
  float y_rpm = 0.0;
  float tan_rpm = 0.0;

  // convert linear vel m/s to m/min
  linear_vel_x_mins = msg.linear.x * 60;
  linear_vel_y_mins = msg.linear.y * 60;

  // convert rad/s to rad/min
  angular_vel_z_mins = msg.angular.z * 60;

  // calculate the tangential velocity
  tangential_vel = angular_vel_z_mins * (base_width_ / 2);

  x_rpm = linear_vel_x_mins / (M_PI * wheel_diameter_);
  y_rpm = linear_vel_y_mins / (M_PI * wheel_diameter_);
  tan_rpm = tangential_vel / (M_PI * wheel_diameter_);

  // calculate the target motors RPM
  // left motor
  vel_target_left_motor_rpm_ =  x_rpm - y_rpm - tan_rpm;
  vel_target_left_motor_rpm_ = constrainFunction( vel_target_left_motor_rpm_, -max_rpm_, max_rpm_);

  // right motor
  vel_target_right_motor_rpm_ = x_rpm + y_rpm + tan_rpm;
  vel_target_right_motor_rpm_ = constrainFunction( vel_target_right_motor_rpm_, -max_rpm_, max_rpm_);

}


void MotorsPidOdom::performPidOdom()
{
  // Calculates the actual wheels RPM velocities based on the ticks
  current_time_ = ros::Time::now();
  delta_time_ = (current_time_ - previous_time_).toSec();
  double delta_time_minute = delta_time_ / 60;
  double left_ticks_delta_ticks = left_motor_ticks_ - previous_left_motor_ticks_;
  double right_ticks_delta_ticks = right_motor_ticks_ - previous_right_motor_ticks_;

  left_motor_vel_rpm_ = (left_ticks_delta_ticks / ticks_per_rev_) / delta_time_minute;
  right_motor_vel_rpm_ = (right_ticks_delta_ticks / ticks_per_rev_) / delta_time_minute;

  // PID controller and calculation of the required PWM to drive the motor
  error_left_ = vel_target_left_motor_rpm_ - left_motor_vel_rpm_;
  integral_left_ += error_left_;
  derivative_left_ = error_left_ - previous_error_left_;
  // Anti WIND UP elaboration
  if (integral_left_ > out_max_) integral_left_ = out_max_;
  else if (integral_left_ < out_min_) integral_left_ = out_min_;
  // -
  if(vel_target_left_motor_rpm_ == 0 && error_left_ == 0)
  {
    integral_left_ = 0;
  }

  error_right_ = vel_target_right_motor_rpm_ - right_motor_vel_rpm_;
  integral_right_ += error_right_;
  derivative_right_ = error_right_ - previous_error_right_;
  // Anti WIND UP elaboration
  if (integral_right_ > out_max_) integral_right_ = out_max_;
  else if (integral_right_ < out_min_) integral_right_ = out_min_;
  //
  if(vel_target_right_motor_rpm_ == 0 && error_right_ == 0)
  {
    integral_right_ = 0;
  }

  // Application of the PID control left
  pwm_left_ = (kp_ * error_left_) + (ki_ * integral_left_) + (kd_ * derivative_left_);

  // Application of the PID control right
  pwm_right_ = (kp_ * error_right_) + (ki_ * integral_right_) + (kd_ * derivative_right_);

  // Saving of the actual errors
  previous_error_left_ = error_left_;
  previous_error_right_ = error_right_;

  // PWM setpoint is constrained between min and max to prevent pid
  // from having too much error
  motor_left_pwm_.data = constrainFunction(pwm_left_, out_min_, out_max_);
  pwm_left_pub_.publish(motor_left_pwm_);

  motor_right_pwm_.data = constrainFunction(pwm_right_, out_min_, out_max_);
  pwm_right_pub_.publish(motor_right_pwm_);

  // Calculate the actual robot velocity
  raw_vel_msg_.header.stamp = current_time_;
  raw_vel_msg_.header.frame_id = "base_link";

  float average_rps_x;
  float average_rps_y;
  float average_rps_a;

  //convert average revolutions per minute to revolutions per second to meter per second along x and y
  average_rps_x = ((left_motor_vel_rpm_ + right_motor_vel_rpm_ ) / 2) / 60; // RPS
  raw_vel_msg_.vector.x  = average_rps_x * (M_PI * wheel_diameter_); // m/s
  raw_vel_msg_.vector.y = 0.0;

  //convert average revolutions per minute to revolutions per second to radiant per second
  average_rps_a = ( (-left_motor_vel_rpm_ + right_motor_vel_rpm_) / 2) / 60;
  raw_vel_msg_.vector.z =  (average_rps_a * (M_PI * wheel_diameter_)) / (base_width_ / 2); //  rad/s

  actual_vel_pub_.publish(raw_vel_msg_);

  double delta_theta;
  delta_theta = raw_vel_msg_.vector.z * delta_time_;

  double delta_x = (raw_vel_msg_.vector.x * cos(theta_pid_) - raw_vel_msg_.vector.y * sin(theta_pid_)) * delta_time_; //m
  double delta_y = (raw_vel_msg_.vector.x * sin(theta_pid_) + raw_vel_msg_.vector.y * cos(theta_pid_)) * delta_time_; //m

  //calculate current position of the robot
  x_pos_ += delta_x;
  y_pos_ += delta_y;
  theta_pid_ += delta_theta ;

  //calculate robot's heading in quarternion angle
  //ROS has a function to calculate yaw in quaternion angle
  odom_quat_.setRPY(0,0,theta_pid_);

  odom_trans_.header.frame_id = "odom";
  odom_trans_.child_frame_id = "base_link";
  odom_trans_.header.stamp = current_time_;
  //robot's position in x,y, and z
  odom_trans_.transform.translation.x = x_pos_;
  odom_trans_.transform.translation.y = y_pos_;
  odom_trans_.transform.translation.z = 0.0;
  //robot's heading in quaternion
  odom_trans_.transform.rotation.x = odom_quat_.x();
  odom_trans_.transform.rotation.y = odom_quat_.y();
  odom_trans_.transform.rotation.z = odom_quat_.z();
  odom_trans_.transform.rotation.w = odom_quat_.w();
  //publish robot's tf using odom_trans object
  if (publish_tf_) tf_br_.sendTransform(odom_trans_);

  odom_.header.stamp = current_time_;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_link";
  //robot's position in x,y, and z
  odom_.pose.pose.position.x = x_pos_;
  odom_.pose.pose.position.y = y_pos_;
  odom_.pose.pose.position.z = 0.0;
  //robot's heading in quaternion
  odom_.pose.pose.orientation.x = odom_quat_.x();
  odom_.pose.pose.orientation.y = odom_quat_.y();
  odom_.pose.pose.orientation.z = odom_quat_.z();
  odom_.pose.pose.orientation.w = odom_quat_.w();
  odom_.pose.covariance[0] = 0.001;
  odom_.pose.covariance[7] = 0.001;
  odom_.pose.covariance[35] = 0.001;
  //linear speed from encoders
  odom_.twist.twist.linear.x = raw_vel_msg_.vector.x;
  odom_.twist.twist.linear.y = raw_vel_msg_.vector.y;
  odom_.twist.twist.linear.z = 0.0;
  // angular speed from encoders
  odom_.twist.twist.angular.x = 0.0;
  odom_.twist.twist.angular.y = 0.0;
  odom_.twist.twist.angular.z = raw_vel_msg_.vector.z;
  odom_.twist.covariance[0] = 0.0001;
  odom_.twist.covariance[7] = 0.0001;
  odom_.twist.covariance[35] = 0.0001;

  odom_pub_.publish(odom_);

  // Saving of the actual ticls count and timing
  previous_time_ = current_time_;
  previous_left_motor_ticks_ = left_motor_ticks_;
  previous_right_motor_ticks_ = right_motor_ticks_;

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "motors_pid_odom_node");

    MotorsPidOdom motors_pid_odom;

    ros::spin();

    return 0;
}
