#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    #include "Wire.h"
#endif

//ROS declarations
#include <ros.h>
#include <ros/time.h>
#include "sensor_msgs/Imu.h"

#define INTERRUPT_PIN 2
#define LED_PIN 13

#define ACC_FACTOR 1 / 16384
#define GYRO_FACTOR  1 / 131
#define MAG_FACTOR  0.3 
#define G_FACTOR 9.81
#define DEG_TO_RAD 0.017453292519943295769236907684886

/******************************************************************************************************** 
* Copyright (C) Antonio Mauro Galiano
* Unauthorized copying of this file, via any medium is strictly prohibited
* Proprietary and confidential
* Written by Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
* December 2021
* Changelog:
*   26-01-2022 Defined new costants for the acc, gyro calculation
*
*********************************************************************************************************/

ros::NodeHandle_<ArduinoHardware, 1, 1, 1, 400> nh;

sensor_msgs::Imu imu_msg; 
ros::Publisher imu_pub("imu/data_raw_6050", &imu_msg);

uint32_t seq;
MPU6050 accelgyro;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t dmpDevStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q; 

//int16_t get_ax, get_ay, get_az, get_gx, get_gy, get_gz;
VectorInt16 acc; 
VectorInt16 gyro; 

bool blinkState = false;

//DMP interrupt detection
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.advertise(imu_pub);

  // initialize device
  nh.loginfo("Initializing I2C IMU device...");
  accelgyro.initialize();
  dmpDevStatus = accelgyro.dmpInitialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // supply your own gyro offsets here, scaled for min sensitivity
  accelgyro.setXAccelOffset(599);
  accelgyro.setYAccelOffset(-671);
  accelgyro.setZAccelOffset(1153);
  accelgyro.setXGyroOffset(61);
  accelgyro.setYGyroOffset(-43);
  accelgyro.setZGyroOffset(22);

  //check the DMP status
  if (dmpDevStatus == 0)
  {

    // turn on the DMP, now that it's ready
    accelgyro.setDMPEnabled(true);
 
    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = accelgyro.getIntStatus();
 
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
 
    // get expected DMP packet size for later comparison
    packetSize = accelgyro.dmpGetFIFOPacketSize();
 
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  
  seq = 0;
}

void loop()
{
  if (!dmpReady) return;

  if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&acc, fifoBuffer);
    accelgyro.dmpGetGyro(&gyro, fifoBuffer);

    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_6050_link";
    imu_msg.header.seq = seq;

    imu_msg.orientation.x = q.x;
    imu_msg.orientation.y = q.y;
    imu_msg.orientation.z = q.z;
    imu_msg.orientation.w = q.w;
  
    imu_msg.linear_acceleration.x = (double)acc.x * ACC_FACTOR * G_FACTOR;
    imu_msg.linear_acceleration.y = (double)acc.y * ACC_FACTOR * G_FACTOR;
    imu_msg.linear_acceleration.z = (double)acc.z * ACC_FACTOR * G_FACTOR;
    
    imu_msg.angular_velocity.x = (double)gyro.x * GYRO_FACTOR * DEG_TO_RAD; // rad / sec
    imu_msg.angular_velocity.y = (double)gyro.y * GYRO_FACTOR * DEG_TO_RAD;
    imu_msg.angular_velocity.z = (double)gyro.z * GYRO_FACTOR * DEG_TO_RAD;
  }

  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  
  imu_pub.publish( &imu_msg );
  seq++;
  nh.spinOnce();
}
