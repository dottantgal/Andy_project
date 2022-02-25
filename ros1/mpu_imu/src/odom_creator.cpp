#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>


class SubNanoAndPubImu
{
public:
    SubNanoAndPubImu()
    {
      zero_orientation_set_ = false;

      // Covariance
      imu_msg_.linear_acceleration_covariance[0] = 0.0;
      imu_msg_.linear_acceleration_covariance[4] = 0.0;
      imu_msg_.linear_acceleration_covariance[8] = 0.0;

      imu_msg_.angular_velocity_covariance[0] = 0.0;
      imu_msg_.angular_velocity_covariance[4] = 0.0;
      imu_msg_.angular_velocity_covariance[8] = 0.0;

      imu_msg_.orientation_covariance[0] = 0.0;
      imu_msg_.orientation_covariance[4] = 0.0;
      imu_msg_.orientation_covariance[8] = 0.0;

      // Traslation relative to world
      transform_.transform.translation.x = 0.0;
      transform_.transform.translation.y = -1.0;
      transform_.transform.translation.z = 1.0;

      pub_imu_ = n_.advertise<sensor_msgs::Imu>("/imu/6050", 1);

      pub_odom_ = n_.advertise<nav_msgs::Odometry>("/odom", 50);

      sub_nano_ = n_.subscribe("/imu/data_raw_6050", 1, &SubNanoAndPubImu::callback, this);

      sub_actual_vel_ = n_.subscribe("/actual_vel", 1, &SubNanoAndPubImu::actualVelCallback, this);


    }


    void actualVelCallback(const geometry_msgs::Vector3Stamped& vel)
    {

      ros::Time current_vel_time = ros::Time::now();
      vel_x_ = vel.vector.x;
      vel_y_ = vel.vector.y;
      vel_z_ = vel.vector.z;
      vel_dt_ = (current_vel_time - last_vel_time_).toSec();
      last_vel_time_ = current_vel_time;

    }


    void callback(const std_msgs::Float32MultiArray::ConstPtr& array)
    {
      ros::Time current_imu_time = ros::Time::now() + ros::Duration(0.0);
      // Get the quaternion from the raw Imu topic
      w_ = array->data[1];
      x_ = array->data[2];
      y_ = array->data[3];
      z_ = array->data[4];

      tf2::Quaternion  orientation_tf2_(x_, y_, z_, w_);

      if (!zero_orientation_set_)
      {
        zero_orientation_tf2_ = orientation_tf2_;
        zero_orientation_set_ = true;
      }
      differential_rotation_tf2_ = zero_orientation_tf2_.inverse() * orientation_tf2_;

      // Applying a rotation to make TF consistent with the MPU real reference
      quaternion_rotation_tf2_.setRPY(roll_, pitch_, yaw_);
      new_quaternion_tf2_ = quaternion_rotation_tf2_ * differential_rotation_tf2_;
      new_quaternion_tf2_.normalize();

      // Imu message header
      imu_msg_.header.stamp = ros::Time::now();
      imu_msg_.header.frame_id = "imu_6050_link";
      imu_msg_.header.seq = array->data[0];

      // Imu message orientation
      tf2::convert(differential_rotation_tf2_, imu_msg_.orientation);

      // Imu message linear acceleration
      imu_msg_.linear_acceleration.x = array->data[5];
      imu_msg_.linear_acceleration.y = array->data[6];
      imu_msg_.linear_acceleration.z = array->data[7];

      // Imu message angular velocity
      imu_msg_.angular_velocity.x = array->data[8]; // rad / sec
      imu_msg_.angular_velocity.y = array->data[9];
      imu_msg_.angular_velocity.z = array->data[10];

      // Publish the Imu real data
      pub_imu_.publish(imu_msg_);

      imu_dt_ = (current_imu_time - last_imu_time_).toSec();
      last_imu_time_ = current_imu_time;

      double delta_theta;
      delta_theta = imu_msg_.angular_velocity.z  * imu_dt_;

      double delta_x = (vel_x_ * cos(theta_pid_) - vel_y_ * sin(theta_pid_)) * vel_dt_; //m
      double delta_y = (vel_x_ * sin(theta_pid_) + vel_y_ * cos(theta_pid_)) * vel_dt_; //m

      x_odom_ += delta_x;
      y_odom_ += delta_y;
      theta_pid_ += delta_theta;

      //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_pid_);

      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_imu_time;
      odom_trans.header.seq = array->data[0];
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = x_odom_;
      odom_trans.transform.translation.y = y_odom_;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = imu_msg_.orientation;
      tf_br_.sendTransform(odom_trans);

      nav_msgs::Odometry odom;
      odom.header.stamp = current_imu_time;
      odom.header.frame_id = "odom";
      //robot's position in x,y, and z
      odom.pose.pose.position.x = x_odom_;
      odom.pose.pose.position.y = y_odom_;
      odom.pose.pose.position.z = 0.0;
      //robot's heading in quaternion
      odom.pose.pose.orientation.x = new_quaternion_tf2_.x();
      odom.pose.pose.orientation.y = new_quaternion_tf2_.y();
      odom.pose.pose.orientation.z = new_quaternion_tf2_.z();
      odom.pose.pose.orientation.w = new_quaternion_tf2_.w();

      odom.child_frame_id = "base_link";
      //linear speed from encoders
      odom.twist.twist.linear.x = vel_x_;
      odom.twist.twist.linear.y = vel_y_;
      odom.twist.twist.linear.z = 0.0;

      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;

      odom.twist.twist.angular.z = imu_msg_.angular_velocity.z ;

      pub_odom_.publish(odom);

    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_odom_;
    ros::Subscriber sub_nano_;
    ros::Subscriber sub_actual_vel_;
    ros::Time transform_time_;
    ros::Time last_vel_time_;
    ros::Time last_imu_time_;
    sensor_msgs::Imu imu_msg_;

    tf2_ros::StaticTransformBroadcaster tf_br_;
    geometry_msgs::TransformStamped transform_;
    tf2::Quaternion orientation_tf2_;
    tf2::Quaternion zero_orientation_tf2_;
    tf2::Quaternion differential_rotation_tf2_;
    tf2::Quaternion quaternion_rotation_tf2_;
    tf2::Quaternion new_quaternion_tf2_;

    bool zero_orientation_set_;
    double w_=0, x_=0, y_=0, z_=0, x_odom_=0, y_odom_=0;
    double vel_x_=0, vel_y_=0, vel_z_=0, vel_dt_=0, imu_dt_=0, theta_pid_ = 0;
    double roll_=0, pitch_=0, yaw_=-M_PI/2;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "imu_type_creator_tf2_6050");

    SubNanoAndPubImu subpubobj;

    ros::spin();

    return 0;
}
