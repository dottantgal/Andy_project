 /**
 *  @file       serial_pid.cpp
 *  @brief      Node to apply the PID control to the velocity of the motor and publish
 *              the PWM directly for the motor driver.
 *              This version is feasible for the use with the ros_lib solution.
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *  @author     Silvano Sallese <silvano.sallese@gmail.com>
 *
 */

#include <dynamic_reconfigure/server.h>
#include <include/low_level/serial_sensor.h>

using namespace std;


/**
 * @brief   Dynamic reconfigure callback.
 * @details Callback for the dynamic reconfiguration of the PID parameters
 *          It will get called when the dynamic_reconfigure server is sent a
 *          new configuration
 *
 * @param config the new config
 *        level a bitmask which is the result of ORing together all of level
 *              values of the parameters that have changed
 */
void callback(andy_ros1::pidConfig &config, uint32_t level)
{
  ROS_DEBUG_STREAM("[serial_pid_node] Reconfigure Request: \n"
    << "Kp gain                     : " << config.Kp << "\n"
    << "Ki gain                     : " << config.Ki << "\n"
    << "Kd gain                     : " << config.Kd << "\n");
  Kp = config.Kp;
  Ki = config.Ki;
  Kd = config.Kd;
}


/**
 * @brief   Constrain function.
 * @details It gets an incoming x value and compares it with a max and min:
 *          it returns max if x>max, min if x<min, x if min<x<max
 *
 * @param x the incoming value
 *        a the minimum value
 *        b the maximum value
 */
int constrain(const double &x, const int &a, const int &b)
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


/**
 * @brief Set rosconsole logger level
 *
 * @param debug True to enable Debug level, false to enable Info level.
 * @return int Status of the operation.
 */
int setLoggerLevel(bool debug)
{
  if (debug) // try to set Debug level
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


/**
 * @brief   Left ticks topic callback.
 * @details Callback to the ticks topic coming from the left motor encoder.
 *
 * @param msg is the message published in the /friday/ticks_left_motor topic.
 */
void leftTicksCallback(const hyperlync_ros::Ticks& left_msg)
{
  left_ticks_enc = left_msg.ticks;
}


/**
 * @brief   Right ticks topic callback.
 * @details Callback to the ticks topic coming from the right motor encoder.
 *
 * @param msg is the message published in the /friday/ticks_right_motor topic.
 */
void rightTicksCallback(const hyperlync_ros::Ticks& right_msg)
{
  right_ticks_enc = right_msg.ticks;
}


/**
 * @brief   Left vel topic callback.
 * @details Callback to the velocity topic coming from the left motor encoder.
 *
 * @param msg is the message published in the /friday/actual_left topic.
 */
void leftVelCallback(const  std_msgs::Float32& left_msg)
{
  left_vel_msg = left_msg.data;
}


/**
 * @brief   Right vel topic callback.
 * @details Callback to the velocity topic coming from the right motor encoder.
 *
 * @param msg is the message published in the /friday/actual_right topic.
 */
void rightVelCallback(const  std_msgs::Float32& right_msg)
{
  right_vel_msg = right_msg.data;
}


/**
 * @brief   Command velocity topic callback.
 * @details Callback to the /cmd_vel topic that set the velocity target for the
 *          robot. It also creates the required linear and tangential RPM
 *
 * @param msg is the message published in the /cmd_vel topic.
 */
void vel_targetCallback(const geometry_msgs::Twist& msg)
{
  float linear_vel_x_mins = 0.0;
  float linear_vel_y_mins = 0.0;
  float angular_vel_z_mins = 0.0;
  float tangential_vel = 0.0;
  float x_rpm = 0.0;
  float y_rpm = 0.0;
  float tan_rpm = 0.0;

  //convert linear vel m/s to m/min
  linear_vel_x_mins = msg.linear.x * 60;
  linear_vel_y_mins = msg.linear.y * 60;

  //convert rad/s to rad/min
  angular_vel_z_mins = msg.angular.z * 60;

  tangential_vel = angular_vel_z_mins * (Base_width / 2);

  x_rpm = linear_vel_x_mins / (M_PI * Wheel_diameter);
  y_rpm = linear_vel_y_mins / (M_PI * Wheel_diameter);
  tan_rpm = tangential_vel / (M_PI * Wheel_diameter);

  // calculate for the target motor RPM and direction
  // left motor
  vel_target_left_rpm =  x_rpm - y_rpm - tan_rpm;
  vel_target_left_rpm = constrain( vel_target_left_rpm, -max_rpm, max_rpm);

  // right motor
  vel_target_right_rpm = x_rpm + y_rpm + tan_rpm;
  vel_target_right_rpm = constrain( vel_target_right_rpm, -max_rpm, max_rpm);

}


/**
 * @brief   Calculation of the PID control
 * @details Function to calculate in real time the PID control for the
 *          left motor, giving as result the PWM value for the motors
 */
void doPID(ros::Publisher *pwm_left_pub, ros::Publisher *pwm_right_pub, ros::Publisher *vel_pub)
{

  // Calculates the actual wheels RPM velocities based on the ticks
  current_time = ros::Time::now();
  delta_time = (current_time - previous_time).toSec();
  double delta_time_minute = delta_time / 60;
  double left_ticks_delta_ticks = left_ticks_enc - left_ticks_prev_encoder;
  double right_ticks_delta_ticks = right_ticks_enc - right_ticks_prev_encoder;

  left_motor_vel_rpm = (left_ticks_delta_ticks / Ticks_per_rev) / delta_time_minute;
  right_motor_vel_rpm = (right_ticks_delta_ticks / Ticks_per_rev) / delta_time_minute;

  // PID controller and calculation of the required PWM to drive the motor
  error_left = vel_target_left_rpm - left_motor_vel_rpm;
  integral_left += error_left;
  derivative_left = error_left - previous_error_left;
  // Anti WIND UP elaboration
  if (integral_left > out_max) integral_left = out_max;
  else if (integral_left < out_min) integral_left = out_min;
  // -
  if(required_rpm_left == 0 && error_left == 0)
  {
    integral_left = 0;
  }

  error_right = vel_target_right_rpm - right_motor_vel_rpm;
  integral_right += error_right;
  derivative_right = error_right - previous_error_right;
  // Anti WIND UP elaboration
  if (integral_right > out_max) integral_right = out_max;
  else if (integral_right < out_min) integral_right = out_min;
  //
  if(required_rpm_right == 0 && error_right == 0)
  {
    integral_right = 0;
  }

  // Application of the PID control left
  pwm_left = (Kp * error_left) + (Ki * integral_left) + (Kd * derivative_left);

  // Application of the PID control right
  pwm_right = (Kp * error_right) + (Ki * integral_right) + (Kd * derivative_right);

  // Saving of the actual errors
  previous_error_left = error_left;
  previous_error_right = error_right;

  // PWM setpoint is constrained between min and max to prevent pid
  // from having too much error
  motor_left_pwm.data = constrain(pwm_left, out_min, out_max);
  pwm_left_pub->publish(motor_left_pwm);
  motor_right_pwm.data = constrain(pwm_right, out_min, out_max);
  pwm_right_pub->publish(motor_right_pwm);

  raw_vel_msg.header.stamp = current_time;
  raw_vel_msg.header.frame_id = "base_link";

  float average_rps_x;
  float average_rps_y;
  float average_rps_a;

  //convert average revolutions per minute to revolutions per second
  average_rps_x = ( (left_motor_vel_rpm + right_motor_vel_rpm ) / 2) / 60; // RPM
  raw_vel_msg.vector.x  = average_rps_x * (M_PI * Wheel_diameter); // m/s

  //convert average revolutions per minute in y axis to revolutions per second
  raw_vel_msg.vector.y = 0.0;

  //convert average revolutions per minute to revolutions per second
  average_rps_a = ( (-left_motor_vel_rpm + right_motor_vel_rpm) / 2) / 60;
  raw_vel_msg.vector.z =  (average_rps_a * (M_PI * Wheel_diameter)) / (Base_width / 2); //  rad/s

  vel_pub->publish(raw_vel_msg);

  // Saving of the actual ticls count and timing
  previous_time = current_time;
  left_ticks_prev_encoder = left_ticks_enc;
  right_ticks_prev_encoder = right_ticks_enc;

//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Required RIGHT wheel rpm = "<<required_rpm_right);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Required LEFT wheel rpm = "<<required_rpm_left);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Actual RIGHT wheel rpm = "<<right_ticks_vel_rpm);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Actual LEFT wheel rpm = "<<left_ticks_vel_rpm);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Motor RIGHT pwm = "<<motor_right_pwm.data);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Motor LEFT pwm = "<<motor_left_pwm.data);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Error RIGHT wheel = "<<error_right);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Error LEFT wheel = "<<error_left);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Integral RIGHT wheel = "<<integral_right);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Integral LEFT wheel = "<<integral_left);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Derivative RIGHT wheel = "<<derivative_right);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Derivative LEFT wheel = "<<derivative_left);

}


/**
 * @brief   Calculation of the linera velocity
 * @details This function publishes the actual and target linear and angular
 *          speeds of the robot as /actual_vel and /target_vel topics
 */
//void calcVelocity(ros::Publisher *vel_pub)
//{
//
//
//  double right_ticks_vel_rad_sec;
//  double left_ticks_vel_rad_sec;
//
//  // Calculation of the linear and angular robot velocities from wheels RPM
//  double right_ticks_vel_minute = right_motor_vel_rpm * (M_PI * Wheel_diameter);
//  double left_ticks_vel_minute = left_motor_vel_rpm * (M_PI * Wheel_diameter);
//
//  double right_ticks_vel_linear = right_ticks_vel_minute / 60;
//  double left_ticks_vel_linear = left_ticks_vel_minute / 60;
//
//  right_ticks_vel_rad_sec = right_ticks_vel_linear / (Wheel_diameter / 2);
//  left_ticks_vel_rad_sec = left_ticks_vel_linear / (Wheel_diameter / 2);
//
//  double linear_velocity = ((Wheel_diameter / 2) * (right_ticks_vel_rad_sec + left_ticks_vel_rad_sec)) / 2;
//  double angular_velocity = ((Wheel_diameter / 2) * (right_ticks_vel_rad_sec - left_ticks_vel_rad_sec)) / Base_width;
//
//  ROS_DEBUG_STREAM(NODE_ID_A
//  << "linear_velocity = "<<linear_velocity);
//  ROS_DEBUG_STREAM(NODE_ID_A
//  << "angular_velocity = "<<angular_velocity);
//
//  // Creation and publishing of the actual robot velocity message
//  raw_vel_msg.header.stamp = ros::Time::now();
//  raw_vel_msg.header.frame_id = "base_link";
//  raw_vel_msg.vector.x = linear_velocity;
//  raw_vel_msg.vector.y = 0.0;
//  raw_vel_msg.vector.z = angular_velocity;
//  vel_pub->publish(raw_vel_msg);
//
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Required robot linear m/s velocity = "<<required_linear_x_vel);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Required robot angular with ratio rad/s velocity = "<<required_angular_vel);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Published robot linear actual m/s velocity = "<<linear_velocity);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Published robot angular actual rad/s velocity = "<<angular_velocity);
//
//}


/**
 * @brief Get the required ROS paramenters.
 *
 * @return int 0 if all is ok. Negative number if something goes wrong.
 */
int getParams()
{
  int res = 0;

  // enable/disable debug lines
  ros::param::get("~debug", DEBUG_PID);

  // set rosconsole level for logging
  res = setLoggerLevel(DEBUG_PID);
  if (res)
    ROS_DEBUG_STREAM(NODE_ID_A << "Debug level enabled");

  res = 0;

  ros::param::get("~Kp", Kp);

  ros::param::get("~Kd", Kd);

  ros::param::get("~Ki", Ki);

  ros::param::get("~Wheel_diameter", Wheel_diameter);

  ros::param::get("~Base_width", Base_width);

  ros::param::get("~Ticks_per_rev", Ticks_per_rev);

  ros::param::get("~out_min", out_min);

  ros::param::get("~out_max", out_max);

  ros::param::get("~max_rpm", max_rpm);

  ros::param::get("~serial_pid_loop_rate", serial_pid_loop_rate);

  return 0;
}

/**
 * @brief   Main function
 * @details Declaration of node handle, publishers, subscribers, dynamic
 *          reconfigure callback and main loop to functions
 */
 int main(int argc, char* argv[])
 {

  // ROS initialization
  ros::init(argc, argv, "serial_pid_node");

  // Nodehandle
  ros::NodeHandle nh_pid;

  // Get Parameters
  int res = getParams();

  if (res != 0)
  {
    ROS_ERROR_STREAM(NODE_ID_A << "Ops, it's not possible to get required parameters!");
  }
  //-

  // Subscriber to velocity target
  ros::Subscriber vel_target_subscriber =
    nh_pid.subscribe("/cmd_vel", 1, vel_targetCallback);

  // Subscriber to the left motor m/s velocity
  ros::Subscriber left_vel_subscriber =
    nh_pid.subscribe("/friday/actual_left", 1, leftVelCallback);

  // Subscriber to the right motor m/s velocity
  ros::Subscriber right_vel_subscriber =
    nh_pid.subscribe("/friday/actual_right", 1, rightVelCallback);

  // Subscriber to the left tick topic OLD
  ros::Subscriber left_tick_subscriber =
    nh_pid.subscribe("/friday/ticks_left_motor", 1, leftTicksCallback);

  // Subscriber to the right tick topic OLD
  ros::Subscriber right_tick_subscriber =
    nh_pid.subscribe("/friday/ticks_right_motor", 1, rightTicksCallback);

  // Publisher for actual PWM values
  // This is the final output of the PID controller that goes to the motor.
  // It's possible to change the range of the PWM output using the out_min and
  // out_max ROS parameter.
  // Left motor PWM publisher OLD
  ros::Publisher pwm_left_publisher =
    nh_pid.advertise<std_msgs::Int16>("/friday/left_pwm_motor",1);

  // Right motor PWM publisher OLD
  ros::Publisher pwm_right_publisher =
    nh_pid.advertise<std_msgs::Int16>("/friday/right_pwm_motor",1);

  // Publisher to actual velocity OLD
  ros::Publisher vel_publisher =
      nh_pid.advertise<geometry_msgs::Vector3Stamped>("/actual_vel", 1);

  // Publisher to target velocity M/S right wheel
  ros::Publisher target_vel_right_pub =
    nh_pid.advertise<std_msgs::Float32>("/friday/target_right", 1);

  // Publisher to target velocity M/S left wheel
  ros::Publisher target_vel_left_pub =
    nh_pid.advertise<std_msgs::Float32>("/friday/target_left", 1);

  // Dynamic reconfigure initialization
  dynamic_reconfigure::Server<andy_ros1::pidConfig> server;
  dynamic_reconfigure::Server<andy_ros1::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Time variables initialization
  current_time = ros::Time::now();
  previous_time = ros::Time::now();

  // Main loop rate
  ros::Rate loop_rate(serial_pid_loop_rate);

  // PWM data initialization
  motor_right_pwm.data = 0;
  motor_left_pwm.data = 0;

//  // This prevent the PWM peak at the beginning due to the absence of
//  // the ticks countings reset at low level fw
//  while(ros::ok())
//  {
//    ros::spinOnce();
//    loop_rate.sleep();
//    left_ticks_prev_encoder = left_ticks_enc;
//    right_ticks_prev_encoder = right_ticks_enc;
//    if ( (required_linear_x_vel != 0) || (required_angular_vel != 0) )
//    {
//      // Performs the PID calculation
//      doPID(&pwm_left_publisher, &pwm_right_publisher, &vel_publisher);
//
//      // Calculation of the actual velocity
//      calcVelocity(&vel_publisher);
//      break;
//    }
//  }

  // Main ROS loop
  while(ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();

    // Performs the PID calculation
    doPID(&pwm_left_publisher, &pwm_right_publisher, &vel_publisher);

    // Calculation of the actual velocity
    //calcVelocity(&vel_publisher);

  }

}
