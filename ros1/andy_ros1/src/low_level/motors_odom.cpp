 /**
 *  @file       motors_odom.cpp
 *  @brief      High level node to create motors odometry from encoders ticks
 *              This version is built up with classes.
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *
 */


#include <include/low_level/motors_odom.h>


MotorsOdom::MotorsOdom()
{
  ROS_INFO_STREAM(NODE_ID_ << "Starting motors odometry node");

  int res = getParams();

  if (res != 0)
  {
    ROS_ERROR_STREAM(NODE_ID_ << "Ops, it's not possible to get required parameters!");
  }

  odom_pub_ = nh_odom_.advertise<nav_msgs::Odometry>("/raw_odom", 1);
  actual_vel_sub_ = nh_odom_.subscribe("/actual_vel", 1, &MotorsOdom::actualVelCallback, this);

  createOdometry();

}


MotorsOdom::~MotorsOdom() {}


int MotorsOdom::setLoggerLevel(bool debug)
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


int MotorsOdom::getParams()
{
  int res = 0;

  // enable/disable debug lines
  ros::param::get("~debug", debug_);

  // set rosconsole level for logging
  res = setLoggerLevel(debug_);
  if (res)
    ROS_DEBUG_STREAM(NODE_ID_ << "Debug level enabled");

  res = 0;

  ros::param::get("~motors_odom_loop_rate", motors_odom_loop_rate_);

  ros::param::get("~publish_tf", publish_tf_);

  return 0;
}


void MotorsOdom::actualVelCallback(const geometry_msgs::Vector3Stamped& vel)
{

  vel_x_ = vel.vector.x;
  vel_y_ = vel.vector.y;
  vel_z_ = vel.vector.z;

}


void MotorsOdom::createOdometry()
{
}



/**
 * @brief   Main function
 * @details Creates the serial sensor pid node called 'serial_sensor_pid_node' and transforms the ticks
 *					values into odometry
 *
 */
int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "motors_odom_node");

    MotorsOdom motors_odom;

    ros::spin();

    return 0;
}
