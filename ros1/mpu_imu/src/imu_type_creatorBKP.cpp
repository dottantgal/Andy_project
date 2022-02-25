#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class SubNanoAndPubImu
{
public:
    SubNanoAndPubImu()
    {
        pub_imu_ = n_.advertise<sensor_msgs::Imu>("/imu/real_data", 1);

        sub_nano_ = n_.subscribe("/imu/data_raw_mpu", 1, &SubNanoAndPubImu::callback, this);
    }

    void callback(const std_msgs::Float32MultiArray::ConstPtr& array)
    {
        int i = 0;

        imu_msg_.header.stamp = ros::Time::now();
        imu_msg_.header.frame_id = "imu_link";
        imu_msg_.header.seq = array->data[2];

        imu_msg_.linear_acceleration.x = array->data[3];
        imu_msg_.linear_acceleration.y = array->data[4];
        imu_msg_.linear_acceleration.z = array->data[5];
        imu_msg_.angular_velocity.x = array->data[6]; // rad / sec
        imu_msg_.angular_velocity.y = array->data[7];
        imu_msg_.angular_velocity.z = array->data[8];

        pub_imu_.publish(imu_msg_);

        static_transformStamped_.header.stamp = imu_msg_.header.stamp;
        static_transformStamped_.header.frame_id = "world";
        static_transformStamped_.header.seq = imu_msg_.header.seq;
        static_transformStamped_.child_frame_id = "imu_link";

        static_transformStamped_.transform.translation.x = 0;
        static_transformStamped_.transform.translation.y = 0;
        static_transformStamped_.transform.translation.z = 0;

        static_broadcaster_.sendTransform(static_transformStamped_);

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
    ros::Subscriber sub_nano_;
    sensor_msgs::Imu imu_msg_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    geometry_msgs::TransformStamped static_transformStamped_;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "imu_type_creator");

    SubNanoAndPubImu subpubobj;

    ros::spin();

    return 0;
}
