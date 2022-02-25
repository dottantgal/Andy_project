// ROS header files 7146 9798 = 2652 ticks
#include <ros.h>
#include <ros/time.h>
#include <andy_ros1/Ticks.h>
#include <andy_ros1/ActualVel.h>
#include <andy_ros1_pid/PID.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

// Teensy library header files
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h> //https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Motor.h>
#include <Kinematics.h>
#include <PID.h>

// Constants definition
#define COMMAND_RATE 20 // hz
#define DEBUG true
#define STOP_ANDY_TIME 400
#define MAX_RPM 146
#define WHEEL_DIAMETER 0.136
#define FR_WHEELS_DISTANCE 0.50
#define LR_WHEELS_DISTANCE 0
#define MIN_PWM -512
#define MAX_PWM 512
#define K_P 0.85 // P constant
#define K_I 0.05 // I constant
#define K_D 0.4 // D constant

Encoder motorL_enc(23, 22, 2652); // create a left motor encoder object
Encoder motorR_enc(20, 21, 2652); // create a right motor encoder object
Controller motorL_controller(3, 12);  // create the left motor object
Controller motorR_controller(4, 30);  // create the right motor object 
PID motorL_pid(MIN_PWM, MAX_PWM, K_P, K_I, K_D);  // create the left motor PID object 
PID motorR_pid(MIN_PWM, MAX_PWM, K_P, K_I, K_D);  // create the right motor PID object  

int left_motor_pwm_value = 0;
int right_motor_pwm_value = 0;

float cmd_vel_linear_x = 0;
float cmd_vel_linear_y = 0;
float cmd_vel_angular_z  = 0;

long positionLeft = 0; // previous read
int incomingPWM = 0;
int current_rpm = 0;
unsigned long ticks_header_seq = 0;
unsigned long cmd_vel_previous_time = 0;

Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE);

//-- ROS declarations --//
// ROS nodehandle
ros::NodeHandle_<ArduinoHardware, 3, 3, 1000, 1000> nh;

// Subscribers callbacks
//void leftMotorPwmCallback(const std_msgs::Int16& left_motor_pwm_msg);
//void rightMotorPwmCallback(const std_msgs::Int16& right_motor_pwm_msg);
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg);
void PIDCallback(const andy_ros1_pid::PID& pid);

// Subscribers to PWM values
//ros::Subscriber<std_msgs::Int16> left_motor_pwm_sub("/andy_ros1/left_motor_pwm", leftMotorPwmCallback);
//ros::Subscriber<std_msgs::Int16> right_motor_pwm_sub("/andy_ros1/right_motor_pwm", rightMotorPwmCallback);

// Subscriber to velocity command
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", cmdVelCallback);

// Subscriber to PID values
ros::Subscriber<andy_ros1_pid::PID> pid_sub("pid", PIDCallback);

// Publishers of motors ticks
//andy_ros1::Ticks left_motor_ticks_msg;
//ros::Publisher left_motor_ticks_pub("/andy_ros1/left_motor_ticks", &left_motor_ticks_msg);
//andy_ros1::Ticks right_motor_ticks_msg;
//ros::Publisher right_motor_ticks_pub("/andy_ros1/right_motor_ticks", &right_motor_ticks_msg);

// Publisher of actual robot velocity
andy_ros1::ActualVel raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
//-- End ROS declarations --//

void setup() 
{

  nh.getHardware()->setBaud(500000);
  nh.initNode();
//  nh.subscribe(left_motor_pwm_sub);
//  nh.subscribe(right_motor_pwm_sub);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(pid_sub);
//  nh.advertise(left_motor_ticks_pub);
//  nh.advertise(right_motor_ticks_pub);
  nh.advertise(raw_vel_pub);

//  left_motor_ticks_msg.ticks = 0;
//  right_motor_ticks_msg.ticks = 0;
  
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("Andy Low Level Connected");
  delay(1);
}

void loop()
{
  static unsigned long prev_control_time = 0;
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {
    controlAndyMoves();

    prev_control_time = millis();
  }

//  if ((millis() - cmd_vel_previous_time) >= STOP_ANDY_TIME)
//  {
//      stopAndy();
//  }

  nh.spinOnce();
}

void PIDCallback(const andy_ros1_pid::PID& pid)
{
    motorL_pid.updateConstants(pid.p, pid.i, pid.d);
    motorR_pid.updateConstants(pid.p, pid.i, pid.d);
}

void controlAndyMoves()
{
    Kinematics::rpm req_rpm = kinematics.getRPM(cmd_vel_linear_x, cmd_vel_linear_y, cmd_vel_angular_z);

    int motorL_rpm = motorL_enc.getRPM();
    int motorR_rpm = motorR_enc.getRPM();

    motorL_controller.spin(motorL_pid.compute(req_rpm.motorL, motorL_rpm));
    motorR_controller.spin(motorR_pid.compute(req_rpm.motorR, motorR_rpm));

    Kinematics::velocities current_vel;

    current_vel = kinematics.getVelocities(motorL_rpm, motorR_rpm);

    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;
    
    raw_vel_pub.publish(&raw_vel_msg);

}


void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
    cmd_vel_linear_x = cmd_vel_msg.linear.x;
    cmd_vel_linear_y = cmd_vel_msg.linear.y;
    cmd_vel_angular_z = cmd_vel_msg.angular.z;

    cmd_vel_previous_time = millis();
}


void stopAndy()
{
    cmd_vel_linear_x = 0.0;
    cmd_vel_linear_y = 0.0;
    cmd_vel_angular_z = 0.0;
}
