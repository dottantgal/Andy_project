 /**
 *  @file       motors_pid.cpp
 *  @brief      High level node to apply the PID control to the velocity of the motor and publish
 *              the PWM directly for the motor driver.
 *              This version is built up with classes.
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *
 */


#include <include/low_level/motors_pid.h>


MotorsPid::MotorsPid()
{
  ROS_INFO_STREAM(NODE_ID_ << "Starting motors PID node");

  f = boost::bind(&MotorsPid::reconfigureCallback, this, _1, _2);
  server.setCallback(f);

  int res = getParams();

  if (res != 0)
  {
    ROS_ERROR_STREAM(NODE_ID_ << "Ops, it's not possible to get required parameters!");
  }

  vel_target_sub_ = nh_pid_.subscribe("/cmd_vel", 1, &MotorsPid::velTargetCallback, this);
  left_ticks_sub_ = nh_pid_.subscribe("/andy_ros1/left_motor_ticks", 1, &MotorsPid::leftTicksCallback, this);
  right_ticks_sub_ = nh_pid_.subscribe("/andy_ros1/right_motor_ticks", 1, &MotorsPid::rightTicksCallback, this);
  pwm_left_pub_ = nh_pid_.advertise<std_msgs::Int16>("/andy_ros1/left_motor_pwm",1);
  pwm_right_pub_ = nh_pid_.advertise<std_msgs::Int16>("/andy_ros1/right_motor_pwm",1);
  actual_vel_pub_ = nh_pid_.advertise<geometry_msgs::Vector3Stamped>("/actual_vel", 1);

  performPid();

}


MotorsPid::~MotorsPid() {}


void MotorsPid::reconfigureCallback(andy_ros1::pidConfig &config, uint32_t level)
{
  ROS_DEBUG_STREAM(NODE_ID_ << "Reconfigure Request: \n"
    << "Kp gain                     : " << config.Kp << "\n"
    << "Ki gain                     : " << config.Ki << "\n"
    << "Kd gain                     : " << config.Kd << "\n");
  kp_ = config.Kp;
  ki_ = config.Ki;
  kd_ = config.Kd;
}


int MotorsPid::constrainFunction(const double &x, const int &a, const int &b)
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


int MotorsPid::setLoggerLevel(bool debug)
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


int MotorsPid::getParams()
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

  ros::param::get("~motors_pid_loop_rate", motors_pid_loop_rate_);

  return 0;
}


void MotorsPid::leftTicksCallback(const andy_ros1::Ticks& left_msg)
{
  left_motor_ticks_ = left_msg.ticks;
}


void MotorsPid::rightTicksCallback(const andy_ros1::Ticks& left_msg)
{
  right_motor_ticks_ = left_msg.ticks;
}


void MotorsPid::velTargetCallback(const geometry_msgs::Twist& msg)
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


void MotorsPid::performPid()
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

  raw_vel_msg_.header.stamp = current_time_;
  raw_vel_msg_.header.frame_id = "base_link";

  float average_rps_x;
  float average_rps_y;
  float average_rps_a;

  //convert average revolutions per minute to revolutions per second
  average_rps_x = ((left_motor_vel_rpm_ + right_motor_vel_rpm_ ) / 2) / 60; // RPM
  raw_vel_msg_.vector.x  = average_rps_x * (M_PI * wheel_diameter_); // m/s

  //convert average revolutions per minute in y axis to revolutions per second
  raw_vel_msg_.vector.y = 0.0;

  //convert average revolutions per minute to revolutions per second
  average_rps_a = ( (-left_motor_vel_rpm_ + right_motor_vel_rpm_) / 2) / 60;
  raw_vel_msg_.vector.z =  (average_rps_a * (M_PI * wheel_diameter_)) / (base_width_ / 2); //  rad/s

  actual_vel_pub_.publish(raw_vel_msg_);

  // Saving of the actual ticls count and timing
  previous_time_ = current_time_;
  previous_left_motor_ticks_ = left_motor_ticks_;
  previous_right_motor_ticks_ = right_motor_ticks_;

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "motors_pid_node");

    MotorsPid motors_pid;

    ros::spin();

    return 0;
}
