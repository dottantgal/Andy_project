#include <include/low_level/serial_sensor.h>


using namespace std;


/**
 * @brief   Callback function
 * @details Callback for the subscription to the robot actual velocity topic
 *
 * @param msg It is the message contained in the /actual_vel topic
 */
void actualVelCallback(const geometry_msgs::Vector3Stamped& vel)
{

  vel_x = vel.vector.x;
  vel_y = vel.vector.y;
  vel_z = vel.vector.z;

}

/**
 * @brief   Callback function
 * @details Callback for the subscription to the Imu topic
 *
 * @param msg It is the message contained in the /friday/imu topic
 */
//void imuCallback(const sensor_msgs::Imu& imu_msg){
//
//  ros::Time current_imu_time = ros::Time::now();
//
//  //Filtering out the imu noise
// 	if( (imu_msg.angular_velocity.z > -0.03) && (imu_msg.angular_velocity.z < 0.03) ){
//    imu_z = 0.00;
//  } else{
//    imu_z = imu_msg.angular_velocity.z;
//  }
//
//  imu_dt = (current_imu_time - last_imu_time).toSec();
//
//	last_imu_time = current_imu_time;
//}

/**
 * @brief   Main function
 * @details Creates the serial sensor pid node called 'serial_sensor_pid_node' and transforms the ticks
 *					values into odometry
 *
 */
int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "serial_sensor_pid_node");

  //Nodehandle and parameter handle
  ros::NodeHandle nh_sensor_pid;

  ros::param::get("~serial_sensor_pid_loop_rate", serial_sensor_pid_loop_rate);

  ros::param::get("~imu_message_serial_sensor", imu_message_serial_sensor);

  ros::param::get("~publish_tf", publish_tf);

  //Setting the loop rate
  ros::Rate loop_rate(serial_sensor_pid_loop_rate);

  // Publishers declaration
  ros::Publisher odom_pub;
  odom_pub = nh_sensor_pid.advertise<nav_msgs::Odometry>("/raw_odom", 50);

  //Subscribers declaration
//  ros::Subscriber imu_sub;
//  imu_sub = nh_sensor_pid.subscribe("/imu/6050", 1, imuCallback);
  ros::Subscriber actual_vel_sub;
  actual_vel_sub = nh_sensor_pid.subscribe("/actual_vel", 1, actualVelCallback);

  //Time declaration
  ros::Time current_time;

  //Tf broadcaster declaration
  tf2_ros::StaticTransformBroadcaster tf_br_;

  double theta_pid = 0.0;

	while(ros::ok())
	{

		ros::spinOnce();

		//Encoders data elaboration starts to get odometry values
		current_time = ros::Time::now();

    //The linear velocity published from the nano/teensy board(vel_x)
    double linear_velocity_x = vel_x;
    double linear_velocity_y = vel_y;
    double angular_velocity = vel_z;

    vel_dt = (current_time - last_vel_time).toSec();
    last_vel_time = current_time;

    double delta_theta;
    delta_theta = angular_velocity * vel_dt;

    double delta_x = (linear_velocity_x * cos(theta_pid) - linear_velocity_y * sin(theta_pid)) * vel_dt; //m
    double delta_y = (linear_velocity_x * sin(theta_pid) + linear_velocity_y * cos(theta_pid)) * vel_dt; //m

    //calculate current position of the robot
    x += delta_x;
    y += delta_y;
    theta_pid += delta_theta ;

    //calculate robot's heading in quarternion angle
    //ROS has a function to calculate yaw in quaternion angle
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0,0,theta_pid);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    //if (publish_tf) tf_br_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;


    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x;
    odom.twist.twist.linear.y = linear_velocity_y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = angular_velocity;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_pub.publish(odom);

		loop_rate.sleep();

  }//while ros::ok

  return 0;

}
