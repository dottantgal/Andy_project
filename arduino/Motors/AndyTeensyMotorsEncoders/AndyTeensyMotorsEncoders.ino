// ROS header files 7146 9798 = 2652 ticks
#include <ros.h>
#include <ros/time.h>
#include <andy_ros1/Ticks.h>
#include <andy_ros1/ResetTicks.h>
#include <std_msgs/Int16.h>

// Teensy library header files
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h> //https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Motor.h>

// Constants definition
#define COMMAND_RATE 10 // Hz
#define DEBUG false
#define LED_PIN 13

Encoder motorL_enc(23, 22, 728); // create a left motor encoder object
Encoder motorR_enc(20, 21, 728); // create a right motor encoder object
Controller motorL_controller(3, 12);  // create the left motor object
Controller motorR_controller(4, 30);  // create the right motor object    

int left_motor_pwm_value;
int right_motor_pwm_value;

long positionLeft = 0; // previous read
int incomingPWM = 0;
int current_rpm = 0;
long int ticks_header_seq = 0;
bool blinkState = false;

//-- ROS declarations --//
// ROS nodehandle
ros::NodeHandle nh;
using andy_ros1::ResetTicks;

// Reset ticks service
void resetTicksCallback(const ResetTicks::Request & req, ResetTicks::Response & res)
{
  if (req.reset == true)
  {
    nh.loginfo("MOTORS TICKS COUNTS RESET");
    motorL_enc.reset();
    motorR_enc.reset();
    res.reset_done = true;
  }
}
ros::ServiceServer<ResetTicks::Request, ResetTicks::Response> reset_ticks_server("/reset_ticks_service", &resetTicksCallback);

// Subscribers callbacks
void leftMotorPwmCallback(const std_msgs::Int16& left_motor_pwm_msg);
void rightMotorPwmCallback(const std_msgs::Int16& right_motor_pwm_msg);

// Subscribers to PWM values
ros::Subscriber<std_msgs::Int16> left_motor_pwm("/andy_ros1/left_motor_pwm", leftMotorPwmCallback);
ros::Subscriber<std_msgs::Int16> right_motor_pwm("/andy_ros1/right_motor_pwm", rightMotorPwmCallback);

// Publishers of motors ticks
andy_ros1::Ticks left_motor_ticks_msg;
ros::Publisher left_motor_ticks_pub("/andy_ros1/left_motor_ticks", &left_motor_ticks_msg);
andy_ros1::Ticks right_motor_ticks_msg;
ros::Publisher right_motor_ticks_pub("/andy_ros1/right_motor_ticks", &right_motor_ticks_msg);
//-- End ROS declarations --//


void setup() 
{

  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.advertiseService(reset_ticks_server); 
  nh.subscribe(left_motor_pwm);
  nh.subscribe(right_motor_pwm);
  nh.advertise(left_motor_ticks_pub);
  nh.advertise(right_motor_ticks_pub);

  left_motor_ticks_msg.ticks = 0;
  right_motor_ticks_msg.ticks = 0;

  left_motor_pwm_value = 0;
  right_motor_pwm_value = 0;
  
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("[low level motors] Motors Encoders Low Level fw started");

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  
  delay(1);
}

void loop()
{
  ticks_header_seq++;
  static unsigned long prev_control_time = 0;
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {
    left_motor_ticks_msg.header.stamp = nh.now();
    left_motor_ticks_msg.header.frame_id = "left_motor_link";
    left_motor_ticks_msg.header.seq = ticks_header_seq;   
    left_motor_ticks_msg.ticks = motorL_enc.read();

    right_motor_ticks_msg.header.stamp = nh.now();
    right_motor_ticks_msg.header.frame_id = "right_motor_link";
    right_motor_ticks_msg.header.seq = ticks_header_seq;   
    right_motor_ticks_msg.ticks = motorR_enc.read();
    
    left_motor_ticks_pub.publish(&left_motor_ticks_msg);
    right_motor_ticks_pub.publish(&right_motor_ticks_msg);

    motorL_controller.spin(left_motor_pwm_value);
    motorR_controller.spin(right_motor_pwm_value);

    prev_control_time = millis();
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

  nh.spinOnce();
}


void leftMotorPwmCallback(const std_msgs::Int16& left_motor_pwm_msg)
{
  if (DEBUG)
  {
    char buffer_left[50];
    sprintf (buffer_left, "[low level motors] Left motor PWM : %d", left_motor_pwm_msg.data);
    nh.loginfo(buffer_left);
  }
  left_motor_pwm_value = left_motor_pwm_msg.data;
}


void rightMotorPwmCallback(const std_msgs::Int16& right_motor_pwm_msg)
{
  if (DEBUG)
  {
    char buffer_right[50];
    sprintf (buffer_right, "[low level motors] Right motor PWM : %d", right_motor_pwm_msg.data);
    nh.loginfo(buffer_right);
  }
  right_motor_pwm_value = right_motor_pwm_msg.data;
}
