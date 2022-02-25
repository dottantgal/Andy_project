#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//ROS declarations
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>

#define INTERRUPT_PIN 2
#define LED_PIN 13

/******************************************************************************************************** 
* Copyright (C) Antonio Mauro Galiano
* Unauthorized copying of this file, via any medium is strictly prohibited
* Proprietary and confidential
* Written by Dr. Eng. Antonio Mauro Galiano <antoniomauro.galiano@gmail.com>
* December 2021
* 
* This version of the firmware works with the inertial sensor MPU6050
* It uses the DMP Digital Motion Processor to extrapolate orientation, acceleration and velocity
* 
* PINOUT:
*         MPU6050       ARDUINO NANO
*         Vcc           3v3
*         GND           GND
*         SCL           A4
*         SDA           A3
*         AD0           GND
*         INT           D2
*         
* Changelog:         
*         
*********************************************************************************************************/

ros::NodeHandle_<ArduinoHardware, 1, 1, 1, 250> nh;

std_msgs::Float32MultiArray imu_msg;
ros::Publisher imu_pub("imu/data_raw_6050", &imu_msg);

uint32_t seq;
MPU6050 accelgyro;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t dmpDevStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; 
VectorInt16 acc; 
VectorInt16 gyro; 

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

  //ROS init and messages
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  imu_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*1);
  imu_msg.layout.dim[0].label = "imu";
  imu_msg.layout.dim[0].size = 11;
  imu_msg.layout.dim[0].stride = 1*11;
  imu_msg.layout.data_offset = 0;
  imu_msg.data = (float *)malloc(sizeof(float)*11);
  imu_msg.data_length = 11;
  nh.advertise(imu_pub);

  // initialize device
  accelgyro.initialize();
  dmpDevStatus = accelgyro.dmpInitialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //nh.loginfo("Testing connections...");
  nh.loginfo(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // supply your own gyro offsets here, scaled for min sensitivity
  accelgyro.setXAccelOffset(879);
  accelgyro.setYAccelOffset(-651);
  accelgyro.setZAccelOffset(1173);
  accelgyro.setXGyroOffset(63);
  accelgyro.setYGyroOffset(-42);
  accelgyro.setZGyroOffset(23);
  
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

  // message sequence
  seq = 0;
}

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    accelgyro.dmpGetQuaternion(&q, fifoBuffer); 
    accelgyro.dmpGetAccel(&acc, fifoBuffer);
    accelgyro.dmpGetGyro(&gyro, fifoBuffer);
    
    imu_msg.data[0]= seq;

    imu_msg.data[1] = q.w;
    imu_msg.data[2] = q.x;
    imu_msg.data[3] = q.y;
    imu_msg.data[4] = q.z;

    //Acc values
    imu_msg.data[5] = ((float)acc.x / 16384.0) * 9.806;
    imu_msg.data[6] = ((float)acc.y / 16384.0) * 9.806;
    imu_msg.data[7] = ((float)acc.z / 16384.0) * 9.806;

    //Gyro values
    imu_msg.data[8] = ((float)gyro.x / 131) * (M_PI / 180); // rad / sec
    imu_msg.data[9] = ((float)gyro.y / 131) * (M_PI / 180);
    imu_msg.data[10] = ((float)gyro.z / 131) * (M_PI / 180);

  }
  imu_pub.publish( &imu_msg );
  seq++;
  nh.spinOnce();
}
