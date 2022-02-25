 /**
 *  @file       imu_type_creator_tf2_6050.cpp
 *  @brief      Node to create IMU message and broadcast the TF
 *
 *  @author     Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
 *
 *  @date       January 2022
 *  @copyright
 *
 */

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


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
        transform_.transform.translation.y = 0.0;
        transform_.transform.translation.z = 0.05;

        pub_imu_ = n_.advertise<sensor_msgs::Imu>("/imu/6050", 1);

        sub_nano_ = n_.subscribe("/imu/data_raw_6050", 1, &SubNanoAndPubImu::callback, this);
    }

    void callback(const std_msgs::Float32MultiArray::ConstPtr& array)
    {
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
        imu_msg_.header.frame_id = "base_link";
        imu_msg_.header.seq = array->data[0];

        // Imu message orientation
        tf2::convert(new_quaternion_tf2_, imu_msg_.orientation);

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

        // Broadcasting the TF2
        transform_.header.stamp = ros::Time::now() + ros::Duration(0.0);
        transform_.header.frame_id = "base_footprint";
        transform_.child_frame_id = "base_link";
        transform_.transform.rotation.x = new_quaternion_tf2_.x();
        transform_.transform.rotation.y = new_quaternion_tf2_.y();
        transform_.transform.rotation.z = new_quaternion_tf2_.z();
        transform_.transform.rotation.w = new_quaternion_tf2_.w();
        //tf_br_.sendTransform(transform_);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_imu_dmp_;
    ros::Subscriber sub_nano_;
    ros::Time transform_time_;
    sensor_msgs::Imu imu_msg_;

    tf2_ros::StaticTransformBroadcaster tf_br_;
    geometry_msgs::TransformStamped transform_;
    tf2::Quaternion orientation_tf2_;
    tf2::Quaternion zero_orientation_tf2_;
    tf2::Quaternion differential_rotation_tf2_;
    tf2::Quaternion quaternion_rotation_tf2_;
    tf2::Quaternion new_quaternion_tf2_;

    bool zero_orientation_set_;
    double w_, x_, y_, z_;
    double roll_=0, pitch_=0, yaw_=0;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "imu_type_creator_tf2_6050");

    SubNanoAndPubImu subpubobj;

    ros::spin();

    return 0;
}
