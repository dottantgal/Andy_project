#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    #include "Wire.h"
#endif

//ROS declarations
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>

#define INTERRUPT_PIN 2
#define LED_PIN 13

#define ACC_FACTOR 1 / 16384
#define GYRO_FACTOR  1 / 131
#define G_FACTOR 9.81
#define DEG_TO_RAD 0.017453292519943295769236907684886

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
*         MPU6050       TEENSY LC
*         Vcc           3v3
*         GND           GND
*         SCL           19
*         SDA           18
*         AD0           GND
*         INT           2
*         
* Changelog:         
*         
*********************************************************************************************************/

ros::NodeHandle_<ArduinoHardware, 1, 1, 1, 300> nh;

std_msgs::Float32MultiArray imu_msg;
ros::Publisher imu_pub("imu/data_raw_6050", &imu_msg);

uint32_t seq;
MPU6050 accelgyro;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t dmpDevStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
int16_t w, x, y, z;

int16_t get_ax, get_ay, get_az, get_gx, get_gy, get_gz;

uint8_t TeensyIntPin = 2;
bool blinkState = false;

//DMP interrupt detection
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
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
  nh.getHardware()->setBaud(500000);
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

  while (!nh.connected())
  {
    nh.spinOnce();
  }


  // initialize device
  //nh.loginfo("Initializing device...");
  nh.loginfo("[low level imu] Initializing IMU MPU6050 device...");
  accelgyro.initialize();
  nh.loginfo(accelgyro.testConnection() ? "[low level imu] MPU6050 connection successful" : "MPU6050 connection failed");
  nh.loginfo("[low level imu] Initializing DMP...");
  dmpDevStatus = accelgyro.dmpInitialize();

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
    nh.loginfo("low level imu] Enabling DMP...");
    accelgyro.setDMPEnabled(true);
 
    // enable Arduino interrupt detection
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = accelgyro.getIntStatus();
 
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
 
    // get expected DMP packet size for later comparison
    packetSize = accelgyro.dmpGetFIFOPacketSize();
 
  }
  else
    nh.loginfo("low level imu] DMP Init failed");

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  nh.loginfo("[low level imu] MPU6050 configuration done");

  //message sequence
  seq = 0;
}

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize);

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();

  // get current FIFO count
  fifoCount = accelgyro.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    accelgyro.resetFIFO();
    nh.loginfo("FIFO overflow!");
  
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();
    
    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);

    accelgyro.getAcceleration(&get_ax, &get_ay, &get_az);
    accelgyro.getRotation(&get_gx, &get_gy, &get_gz);
    
    fifoCount -= packetSize;
    
    //Quaternion  values
//    imu_msg.data[1] = fifoBuffer[0];
//    imu_msg.data[2] = fifoBuffer[1];
//    imu_msg.data[3] = fifoBuffer[4];
//    imu_msg.data[4] = fifoBuffer[5];
//    imu_msg.data[5] = fifoBuffer[8];
//    imu_msg.data[6] = fifoBuffer[9];
//    imu_msg.data[7] = fifoBuffer[12];
//    imu_msg.data[8] = fifoBuffer[13];

    imu_msg.data[0]= seq;
    
    w = (((0xff &(char)fifoBuffer[0]) << 8) | (0xff &(char)fifoBuffer[1]));
    x = (((0xff &(char)fifoBuffer[4]) << 8) | (0xff &(char)fifoBuffer[5]));
    y = (((0xff &(char)fifoBuffer[8]) << 8) | (0xff &(char)fifoBuffer[9]));
    z = (((0xff &(char)fifoBuffer[12]) << 8) | (0xff &(char)fifoBuffer[13]));

    imu_msg.data[1] = w;
    imu_msg.data[2] = x;
    imu_msg.data[3] = y;
    imu_msg.data[4] = z;

    //Acc values
    imu_msg.data[5] = get_ax * (double) ACC_FACTOR * G_FACTOR;
    imu_msg.data[6] = get_ay * (double) ACC_FACTOR * G_FACTOR;
    imu_msg.data[7] = get_ay * (double) ACC_FACTOR * G_FACTOR;

    //Gyro values
    imu_msg.data[8] = get_gx * (double) GYRO_FACTOR * DEG_TO_RAD; // rad / sec
    imu_msg.data[9] = get_gy * (double) GYRO_FACTOR * DEG_TO_RAD;
    imu_msg.data[10] = get_gz * (double) GYRO_FACTOR * DEG_TO_RAD;

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  imu_pub.publish( &imu_msg );
  seq++;
  nh.spinOnce();
}
