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
void leftTicksCallback(const andy_ros1::Ticks& left_msg)
{
  left_ticks_enc = left_msg.ticks;
}


/**
 * @brief   Right ticks topic callback.
 * @details Callback to the ticks topic coming from the right motor encoder.
 *
 * @param msg is the message published in the /friday/ticks_right_motor topic.
 */
void rightTicksCallback(const andy_ros1::Ticks& right_msg)
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

  required_linear_x_vel = msg.linear.x;
  required_linear_y_vel = msg.linear.y;
  required_angular_vel = msg.angular.z + (msg.angular.z * rotation_ratio);

  // Target RPM velocity calculation
  // it's based on the monocycle equations where is calculated the
  // required RPM for the single wheels starting from the Vt and ω coming
  // from the /cmd_vel target velocity

  // Angular velocities of the wheels rad/sec
  vel_target_right_rad_sec = ((2 * required_linear_x_vel) + (required_angular_vel * Base_width))
                            / ( 2 * (Wheel_diameter / 2));
  vel_target_left_rad_sec  = ((2 * required_linear_x_vel) + (-1.0 * required_angular_vel * Base_width))
                            / ( 2 * (Wheel_diameter / 2));

  // Linear velocity of the wheels m/s
  vel_target_right_linear = (vel_target_right_rad_sec * (Wheel_diameter / 2));
  vel_target_left_linear = (vel_target_left_rad_sec * (Wheel_diameter / 2));

  // Linear velocity of the wheels m/minutes
  vel_target_right_minute = (vel_target_right_linear * 60);
  vel_target_left_minute = (vel_target_left_linear * 60);

  // Revolution per minute of the wheels
  vel_target_right_rpm =  vel_target_right_minute / (M_PI * Wheel_diameter);
  vel_target_left_rpm =  vel_target_left_minute / (M_PI * Wheel_diameter);

}


/**
 * @brief   Calculation of the PID control
 * @details Function to calculate in real time the PID control for the
 *          left motor, giving as result the PWM value for the motors
 */
void doPID(ros::Publisher *pwm_left_pub, ros::Publisher *pwm_right_pub)
{

  // Calculates the actual wheels RPM velocities based on the ticks
  ticks_current_time = ros::Time::now();
  ticks_dt = (ticks_current_time - ticks_then).toSec();
  double ticks_dtm = ticks_dt / 60;
  double right_ticks_delta_ticks = right_ticks_enc - right_ticks_prev_encoder;
  double left_ticks_delta_ticks = left_ticks_enc - left_ticks_prev_encoder;
  double DistancePerTick = ((M_PI * Wheel_diameter) / Ticks_per_rev);

  // Actual real wheels linear velocities Meter/minute
  right_ticks_vel = (right_ticks_delta_ticks * DistancePerTick ) / ticks_dtm;
  left_ticks_vel = (left_ticks_delta_ticks * DistancePerTick) / ticks_dtm;

  // Actual real wheels RPM
  right_ticks_vel_rpm = right_ticks_vel / (M_PI * Wheel_diameter);
  left_ticks_vel_rpm = left_ticks_vel / (M_PI * Wheel_diameter);

  // Calculates the required RPM for the motors based on the actual velocity
  // Required_rpm is constrained to prevent pid from having too much error
  required_rpm_left = constrain( vel_target_left_rpm, -max_rpm, max_rpm);
  required_rpm_right = constrain( vel_target_right_rpm, -max_rpm, max_rpm);

  // PID controller and calculation of the required PWM to drive the motor
  error_left = required_rpm_left - left_ticks_vel_rpm;
  integral_left += (Ki * error_left);
  // Anti WIND UP elaboration
  if (integral_left > out_max) integral_left = out_max;
  else if (integral_left < out_min) integral_left = out_min;
  // -
  derivative_left = error_left - last_input_left;

  error_right = required_rpm_right - right_ticks_vel_rpm;
  integral_right += (Ki * error_right);
  // Anti WIND UP elaboration
  if (integral_right > out_max) integral_right = out_max;
  else if (integral_right < out_min) integral_right = out_min;
  // -
  derivative_right = error_right - last_input_right;

  if(required_rpm_left == 0 && error_left == 0)
  {
    integral_left = 0;
  }

  if(required_rpm_right == 0 && error_right == 0)
  {
    integral_right = 0;
  }

  // Application of the PID control left
  pwm_left = (Kp * error_left) + integral_left - (Kd * derivative_left);

  // Application of the PID control right
  pwm_right = (Kp * error_right) + integral_right - (Kd * derivative_right);

  // Saving of the actual errors
  previous_left_pid_error = error_left;
  previous_right_pid_error = error_right;

  // Saving of the actual inputs
  last_input_right = right_ticks_vel_rpm;
  last_input_left = left_ticks_vel_rpm;

  // PWM setpoint is constrained between min and max to prevent pid
  // from having too much error
  motor_left_pwm.data = constrain(pwm_left, out_min, out_max);
  pwm_left_pub->publish(motor_left_pwm);
  motor_right_pwm.data = constrain(pwm_right, out_min, out_max);
  pwm_right_pub->publish(motor_right_pwm);

  // Saving of the actual ticls count and timing
  ticks_then = ticks_current_time;
  left_ticks_prev_encoder = left_ticks_enc;
  right_ticks_prev_encoder = right_ticks_enc;
//
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
void calcVelocity(ros::Publisher *vel_pub)
{
  double right_ticks_vel_rad_sec;
  double left_ticks_vel_rad_sec;

  // Calculation of the linear and angular robot velocities from wheels RPM
  double right_ticks_vel_minute = right_ticks_vel_rpm * (M_PI * Wheel_diameter);
  double left_ticks_vel_minute = left_ticks_vel_rpm * (M_PI * Wheel_diameter);

  double right_ticks_vel_linear = right_ticks_vel_minute / 60;
  double left_ticks_vel_linear = left_ticks_vel_minute / 60;

  right_ticks_vel_rad_sec = right_ticks_vel_linear / (Wheel_diameter / 2);
  left_ticks_vel_rad_sec = left_ticks_vel_linear / (Wheel_diameter / 2);

  double linear_velocity = ((Wheel_diameter / 2) * (right_ticks_vel_rad_sec + left_ticks_vel_rad_sec)) / 2;
  double angular_velocity = ((Wheel_diameter / 2) * (right_ticks_vel_rad_sec - left_ticks_vel_rad_sec)) / Base_width;

//  ROS_DEBUG_STREAM(NODE_ID_A
//  << "linear_velocity = "<<linear_velocity);
//  ROS_DEBUG_STREAM(NODE_ID_A
//  << "angular_velocity = "<<angular_velocity);

  // Creation and publishing of the actual robot velocity message
  raw_vel_msg.header.stamp = ros::Time::now();
  raw_vel_msg.header.frame_id = "base_link";
  raw_vel_msg.vector.x = linear_velocity;
  raw_vel_msg.vector.y = 0.0;
  raw_vel_msg.vector.z = angular_velocity;
  vel_pub->publish(raw_vel_msg);

//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Required robot linear m/s velocity = "<<required_linear_x_vel);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Required robot angular with ratio rad/s velocity = "<<required_angular_vel);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Published robot linear actual m/s velocity = "<<linear_velocity);
//  ROS_DEBUG_STREAM(NODE_ID_A
//    << "Published robot angular actual rad/s velocity = "<<angular_velocity);

}


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

  ros::param::get("~rotation_ratio", rotation_ratio);

  if (res)
  {
    if (rotation_ratio > 1.00)
    {
      rotation_ratio = 1.00;
      ROS_DEBUG_STREAM( NODE_ID_A << "Rotation ratio for Serial PID node corrected in " << rotation_ratio);
    }
    else if (rotation_ratio < 0.0)
    {
      rotation_ratio = 0.0;
      ROS_DEBUG_STREAM( NODE_ID_A << "Rotation ratio for Serial PID node corrected in " << rotation_ratio);
    }

  }

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
    nh_pid.subscribe("/andy_ros1/actual_left", 1, leftVelCallback);

  // Subscriber to the right motor m/s velocity
  ros::Subscriber right_vel_subscriber =
    nh_pid.subscribe("/andy_ros1/actual_right", 1, rightVelCallback);

  // Subscriber to the left tick topic OLD
  ros::Subscriber left_tick_subscriber =
    nh_pid.subscribe("/andy_ros1/left_motor_ticks", 1, leftTicksCallback);

  // Subscriber to the right tick topic OLD
  ros::Subscriber right_tick_subscriber =
    nh_pid.subscribe("/andy_ros1/right_motor_ticks", 1, rightTicksCallback);

  // Publisher for actual PWM values
  // This is the final output of the PID controller that goes to the motor.
  // It's possible to change the range of the PWM output using the out_min and
  // out_max ROS parameter.
  // Left motor PWM publisher OLD
  ros::Publisher pwm_left_publisher =
    nh_pid.advertise<std_msgs::Int16>("/andy_ros1/left_motor_pwm",1);

  // Right motor PWM publisher OLD
  ros::Publisher pwm_right_publisher =
    nh_pid.advertise<std_msgs::Int16>("/andy_ros1/right_motor_pwm",1);

  // Publisher to actual velocity OLD
  ros::Publisher vel_publisher =
      nh_pid.advertise<geometry_msgs::Vector3Stamped>("/actual_vel", 1);

  // Publisher to target velocity M/S right wheel
  ros::Publisher target_vel_right_pub =
    nh_pid.advertise<std_msgs::Float32>("/andy_ros1/target_right", 1);

  // Publisher to target velocity M/S left wheel
  ros::Publisher target_vel_left_pub =
    nh_pid.advertise<std_msgs::Float32>("/andy_ros1/target_left", 1);

  // Dynamic reconfigure initialization
  dynamic_reconfigure::Server<andy_ros1::pidConfig> server;
  dynamic_reconfigure::Server<andy_ros1::pidConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Time variables initialization
  ticks_current_time = ros::Time::now();
  ticks_then = ros::Time::now();

  // Main loop rate
  ros::Rate loop_rate(serial_pid_loop_rate);

  // PWM data initialization
  motor_right_pwm.data = 0;
  motor_left_pwm.data = 0;

  // This prevent the PWM peak at the beginning due to the absence of
  // the ticks countings reset at low level fw
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    left_ticks_prev_encoder = left_ticks_enc;
    right_ticks_prev_encoder = right_ticks_enc;
    if ( (required_linear_x_vel != 0) || (required_angular_vel != 0) )
    {
      // Performs the PID calculation
      doPID(&pwm_left_publisher, &pwm_right_publisher);

      // Calculation of the actual velocity
      calcVelocity(&vel_publisher);
      break;
    }
  }

  // Main ROS loop
  while(ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();

    // Performs the PID calculation
    doPID(&pwm_left_publisher, &pwm_right_publisher);

    // Calculation of the actual velocity
    calcVelocity(&vel_publisher);

  }

}

