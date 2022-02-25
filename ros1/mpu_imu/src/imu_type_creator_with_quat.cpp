#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


class SubNanoAndPubImu
{
public:
    SubNanoAndPubImu()
    {
        zero_orientation_set_ = false;

        imu_msg_.linear_acceleration_covariance[0] = 0.0;
        imu_msg_.linear_acceleration_covariance[4] = 0.0;
        imu_msg_.linear_acceleration_covariance[8] = 0.0;

        imu_msg_.angular_velocity_covariance[0] = 0.0;
        imu_msg_.angular_velocity_covariance[4] = 0.0;
        imu_msg_.angular_velocity_covariance[8] = 0.0;

        imu_msg_.orientation_covariance[0] = 0.0;
        imu_msg_.orientation_covariance[4] = 0.0;
        imu_msg_.orientation_covariance[8] = 0.0;

        transform_.setOrigin(tf::Vector3(0.0,1.0,1.0));

        pub_imu_ = n_.advertise<sensor_msgs::Imu>("/imu/data", 1);

        sub_nano_ = n_.subscribe("/imu/data_raw_mpu", 1, &SubNanoAndPubImu::callback, this);
    }

    void callback(const std_msgs::Float32MultiArray::ConstPtr& array)
    {
        //CODE IF NANO SENT ALL DATA
//        int16_t w = (((0xff &(char)array->data[1]) << 8) | 0xff &(char)array->data[2]);
//        int16_t x = (((0xff &(char)array->data[3]) << 8) | 0xff &(char)array->data[4]);
//        int16_t y = (((0xff &(char)array->data[5]) << 8) | 0xff &(char)array->data[6]);
//        int16_t z = (((0xff &(char)array->data[7]) << 8) | 0xff &(char)array->data[8]);
//
//        double wf = w/16384.0;
//        double xf = x/16384.0;
//        double yf = y/16384.0;
//        double zf = z/16384.0;

        //CODE IF NANO SENT W X Y Z
        // Get the quaternion from the raw Imu topic
        w_ = array->data[1]/16384.0;
        x_ = array->data[2]/16384.0;
        y_ = array->data[3]/16384.0;
        z_ = array->data[4]/16384.0;
        tf::Quaternion orientation_(x_, y_, z_, w_);
        if (!zero_orientation_set_)
        {
          zero_orientation_ = orientation_;
          zero_orientation_set_ = true;
        }
        differential_rotation_ = zero_orientation_.inverse() * orientation_;

        // Imu message header
        imu_msg_.header.stamp = ros::Time::now();
        imu_msg_.header.frame_id = "imu_link";
        imu_msg_.header.seq = array->data[0];

        // Imu message orientation
        quaternionTFToMsg(differential_rotation_, imu_msg_.orientation);

        //CODE IF NANO SENT ALL DATA
//        //Linear acceleration
//        //We have the values in g, it should be in m/s^2, so * 9.806
//        imu_msg_.linear_acceleration.x = array->data[9];
//        imu_msg_.linear_acceleration.y = array->data[10];
//        imu_msg_.linear_acceleration.z = array->data[11];
//
//        //Angular velocity
//        imu_msg_.angular_velocity.x = array->data[12]; // rad / sec
//        imu_msg_.angular_velocity.y = array->data[13];
//        imu_msg_.angular_velocity.z = array->data[14];

        //CODE IF NANO SENT W X Y Z
        // Imu message Linear acceleration
        imu_msg_.linear_acceleration.x = array->data[5];
        imu_msg_.linear_acceleration.y = array->data[6];
        imu_msg_.linear_acceleration.z = array->data[7];

        // Imu message angular velocity
        imu_msg_.angular_velocity.x = array->data[8]; // rad / sec
        imu_msg_.angular_velocity.y = array->data[9];
        imu_msg_.angular_velocity.z = array->data[10];

        // Publish the Imu real data
        pub_imu_.publish(imu_msg_);

        // Broadcasting the TF
        transform_time_ = ros::Time::now() + ros::Duration(0.0);
        transform_.setRotation(differential_rotation_);
        tf_br_.sendTransform(tf::StampedTransform(transform_, transform_time_, "imu_base",  "imu_link"));
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_imu_dmp_;
    ros::Subscriber sub_nano_;
    ros::Time transform_time_;
    sensor_msgs::Imu imu_msg_;
    tf::TransformBroadcaster tf_br_;
    tf::Transform transform_;
    tf::Quaternion zero_orientation_;
    tf::Quaternion differential_rotation_;
    bool zero_orientation_set_;
    double w_, x_, y_, z_;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "imu_type_creator_with_quat");

    SubNanoAndPubImu subpubobj;

    ros::spin();

    return 0;
}
