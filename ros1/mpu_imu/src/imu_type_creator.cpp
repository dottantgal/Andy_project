#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>


class SubNanoAndPubImu
{
public:
    SubNanoAndPubImu()
    {
        //transform_.setOrigin(tf2::Vector3(0,0,0));
        zero_orientation_ = tf::createQuaternionFromRPY(0.0,0.0,0.0);
        transform_.setOrigin(tf::Vector3(0,0,0));
        pub_imu_ = n_.advertise<sensor_msgs::Imu>("/imu/data", 1);
        //pub_imu_dmp_ = n_.advertise<sensor_msgs::Imu>("/imu/dmp_data", 1);

        sub_nano_ = n_.subscribe("/imu/data_raw_mpu", 1, &SubNanoAndPubImu::callback, this);
    }

    void callback(const std_msgs::Float32MultiArray::ConstPtr& array)
    {
        imu_msg_.header.stamp = ros::Time::now();
        imu_msg_.header.frame_id = "imu_link";
        imu_msg_.header.seq = array->data[0];

        //Linear acceleration
        //We have the values in g, it should be in m/s^2, so * 9.806
        imu_msg_.linear_acceleration.x = array->data[1];
        imu_msg_.linear_acceleration.y = array->data[2];
        imu_msg_.linear_acceleration.z = array->data[3];

        //Angular velocity
        imu_msg_.angular_velocity.x = array->data[4]; // rad / sec
        imu_msg_.angular_velocity.y = array->data[5];
        imu_msg_.angular_velocity.z = array->data[6];

//        orientation_ = tf::createQuaternionFromRPY(array->data[7]*0.1*M_PI/180.0, array->data[8]*0.1*M_PI/180.0, array->data[9]*0.1*M_PI/180.0);
//        differential_rotation_ = zero_orientation_.inverse()*orientation_;
//        quaternionTFToMsg(differential_rotation_, imu_msg_.orientation);
//
//        measurement_time_ = ros::Time::now() + ros::Duration(0.0);
//        transform_.setRotation(differential_rotation_);
//        tf_br_.sendTransform(tf::StampedTransform(transform_, measurement_time_, "imu_base", "imu_link"));
//        imu_msg_.orientation.x = ( sin(array->data[7]/2) * cos(array->data[8]/2) * cos(array->data[9]/2) ) - ( cos(array->data[7]/2) * sin(array->data[8]/2) * sin(array->data[9]/2) );
//        imu_msg_.orientation.y = ( cos(array->data[7]/2) * sin(array->data[8]/2) * cos(array->data[9]/2) ) + ( sin(array->data[7]/2) * cos(array->data[8]/2) * sin(array->data[9]/2) );
//        imu_msg_.orientation.z = ( cos(array->data[7]/2) * cos(array->data[8]/2) * sin(array->data[9]/2) ) - ( sin(array->data[7]/2) * sin(array->data[8]/2) * cos(array->data[9]/2) );
//        imu_msg_.orientation.w = ( cos(array->data[7]/2) * cos(array->data[8]/2) * cos(array->data[9]/2) ) + ( sin(array->data[7]/2) * sin(array->data[8]/2) * sin(array->data[9]/2) );
//
//        imu_msg_.orientation_covariance[0] = 0.0;
//        imu_msg_.orientation_covariance[4] = 0.0;
//        imu_msg_.orientation_covariance[8] = 0.0;

        pub_imu_.publish(imu_msg_);

        //If you wanna pub along with DMP data
//        imu_msg_.header.stamp = imu_msg_dmp_.header.stamp = ros::Time::now();
//        imu_msg_.header.frame_id = imu_msg_dmp_.header.frame_id = "imu_link";
//        imu_msg_.header.seq = imu_msg_dmp_.header.seq = array->data[0];

//        //Get quaternion values
//        tf2::Quaternion orientation_(array->data[2], array->data[3], array->data[4], array->data[1]);
//        zero_orientation_ = orientation_;
//        tf2::Quaternion differential_rotation_;
//        differential_rotation_ = zero_orientation_.inverse() * orientation_;
//        convert(differential_rotation_, imu_msg_.orientation);

//        //Gyro values
//        imu_msg_dmp_.linear_acceleration.x = array->data[8];
//        imu_msg_dmp_.linear_acceleration.y = array->data[9];
//        imu_msg_dmp_.linear_acceleration.z = array->data[10];
//
//        //Acc values
//        imu_msg_dmp_.angular_velocity.x = array->data[5]; // rad / sec
//        imu_msg_dmp_.angular_velocity.y = array->data[6];
//        imu_msg_dmp_.angular_velocity.z = array->data[7];
//
//        //Setting the covarinces
//        imu_msg_dmp_.linear_acceleration_covariance[0] = 0.0;
//        imu_msg_dmp_.linear_acceleration_covariance[4] = 0.0;
//        imu_msg_dmp_.linear_acceleration_covariance[8] = 0.0;
//
//        imu_msg_dmp_.angular_velocity_covariance[0] = 0.0;
//        imu_msg_dmp_.angular_velocity_covariance[4] = 0.0;
//        imu_msg_dmp_.angular_velocity_covariance[8] = 0.0;
//
//        imu_msg_dmp_.orientation_covariance[0] = 0.0;
//        imu_msg_dmp_.orientation_covariance[4] = 0.0;
//        imu_msg_dmp_.orientation_covariance[8] = 0.0;
//
//        imu_msg_.linear_acceleration.x = array->data[11];
//        imu_msg_.linear_acceleration.y = array->data[12];
//        imu_msg_.linear_acceleration.z = array->data[13];
//
//        //Acc values
//        imu_msg_.angular_velocity.x = array->data[14]; // rad / sec
//        imu_msg_.angular_velocity.y = array->data[15];
//        imu_msg_.angular_velocity.z = array->data[16];
//
//        //Setting the covarinces
//        imu_msg_.linear_acceleration_covariance[0] = 0.0;
//        imu_msg_.linear_acceleration_covariance[4] = 0.0;
//        imu_msg_.linear_acceleration_covariance[8] = 0.0;
//
//        imu_msg_.angular_velocity_covariance[0] = 0.0;
//        imu_msg_.angular_velocity_covariance[4] = 0.0;
//        imu_msg_.angular_velocity_covariance[8] = 0.0;
//
//        imu_msg_.orientation_covariance[0] = 0.0;
//        imu_msg_.orientation_covariance[4] = 0.0;
//        imu_msg_.orientation_covariance[8] = 0.0;
//        pub_imu_dmp_.publish(imu_msg_dmp_);

//        static_transformStamped_.header.stamp = ros::Time::now();
//        static_transformStamped_.header.frame_id = "world";
//        static_transformStamped_.child_frame_id = imu_msg_.header.frame_id;
//        static_transformStamped_.transform.translation.x = 0.0;
//        static_transformStamped_.transform.translation.y = 0.0;
//        static_transformStamped_.transform.translation.z = 0.0;
//        static_transformStamped_.transform.rotation = imu_msg_.orientation;
//        static_broadcaster_.sendTransform(static_transformStamped_);

//        static_transformStamped_.header.stamp = imu_msg_.header.stamp;
//        static_transformStamped_.header.frame_id = "world";
//        static_transformStamped_.header.seq = imu_msg_.header.seq;
//        static_transformStamped_.child_frame_id = "imu_link";
//
//        static_transformStamped_.transform.translation.x = 0;
//        static_transformStamped_.transform.translation.y = 0;
//        static_transformStamped_.transform.translation.z = 0;
//
//        static_broadcaster_.sendTransform(static_transformStamped_);

//        // test method to printout the array values
//        for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
//        {
//                ROS_INFO_STREAM("I heard: " << *it << " at position " << i);
//
//
//                i++;
//        }
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_imu_dmp_;
    ros::Subscriber sub_nano_;
    ros::Time measurement_time_;
    sensor_msgs::Imu imu_msg_;
    sensor_msgs::Imu imu_msg_dmp_;
    tf::Quaternion orientation_;
    tf::Quaternion differential_rotation_;
    tf::Quaternion zero_orientation_;
    tf::Transform transform_;
    tf::TransformBroadcaster tf_br_;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "imu_type_creator");

    SubNanoAndPubImu subpubobj;

    ros::spin();

    return 0;
}
