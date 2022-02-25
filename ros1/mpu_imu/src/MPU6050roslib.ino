#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <ros.h>
#include <ros/time.h>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

/******************************************************************************************************** 
* Copyright (C) Hyperlync Technologies Ltd.
* Unauthorized copying of this file, via any medium is strictly prohibited
* Proprietary and confidential
* Written by Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
*             Silvano Sallese <silvano.sallese@gmail.com>
* September 2017
* Changelog:
*      2017-09-21 Updated with the calibration and scale values
*                 Conversion of the angular velocity from degrees/sec to rad/sec
*********************************************************************************************************/

ros::NodeHandle  nh;

sensor_msgs::Imu  imu_msg;
ros::Publisher imu_pub("imu/data_raw_mpu", &imu_msg);

uint32_t seq;
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
const float pi = 3.14159265359;

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(imu_pub);

  // initialize device
  nh.loginfo("Initializing I2C IMU device...");
  accelgyro.initialize();

  // verify connection
  nh.loginfo("Testing device connections...");
  nh.loginfo(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // supply your own gyro offsets here, scaled for min sensitivity
  accelgyro.setXAccelOffset(824);
  accelgyro.setYAccelOffset(-677);
  accelgyro.setZAccelOffset(1169);
  accelgyro.setXGyroOffset(67);
  accelgyro.setYGyroOffset(-35);
  accelgyro.setZGyroOffset(20);
  
  seq = 0;
}

void loop()
{
  seq++;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";
  imu_msg.header.seq = seq;

  imu_msg.linear_acceleration.x = (float)ax / 16384;
  imu_msg.linear_acceleration.y = (float)ay / 16384;
  imu_msg.linear_acceleration.z = (float)az / 16384;
  imu_msg.angular_velocity.x = ((float)gx / 131) * (pi / 180); // rad / sec
  imu_msg.angular_velocity.y = ((float)gy / 131) * (pi / 180);
  imu_msg.angular_velocity.z = ((float)gz / 131) * (pi / 180);

//   Introducing the heading from the magnetometer. When it will be available
//   the zeros will be replaced by the getHeading function 
//   magnetic field * magnitude scale * uTesla_to_Tesla
//   imu_msg.magnetic_field.x = 0 *  0.3 * 0.000001;
//   imu_msg.magnetic_field.y = 0 *  0.3 * 0.000001;
//   imu_msg.magnetic_field.z = 0 *  0.3 * 0.000001;

  imu_pub.publish( &imu_msg );
  nh.spinOnce();
  delay(52); // for the 10Hz topic publishing
}
