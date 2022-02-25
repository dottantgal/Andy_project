/*
  Copyright (c) 2016, Juan Jimeno
  Source: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
   Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
   Neither the name of  nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific
  prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORTPPIPI (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, 
float wheels_x_distance):
    max_rpm_(motor_max_rpm),
    wheels_x_distance_(wheels_x_distance),
    wheel_circumference_(PI * wheel_diameter)
{    
}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{
    float required_linear_x_vel;
    float required_angular_vel;
    float vel_target_left_rad_sec;
    float vel_target_right_rad_sec;
    float vel_target_left_linear;
    float vel_target_right_linear;
    float vel_target_left_minute;
    float vel_target_right_minute;
    float vel_target_left_rpm;
    float vel_target_right_rpm;

    //convert m/s to m/min
    required_linear_x_vel = linear_x;

    //convert rad/s to rad/min
    required_angular_vel = angular_z + (angular_z * 0.5);
    
    // Angular velocities of the wheels rad/sec
		vel_target_right_rad_sec = ((2 * required_linear_x_vel) + (required_angular_vel * 0.5))
				                      / ( 2 * (0.136 / 2));
		vel_target_left_rad_sec  = ((2 * required_linear_x_vel) + (-1.0 * required_angular_vel * 0.5))
				                      / ( 2 * (0.136 / 2));

		// Linear velocity of the wheels m/s
		vel_target_right_linear = (vel_target_right_rad_sec * (0.136 / 2));
		vel_target_left_linear = (vel_target_left_rad_sec * (0.136 / 2));

		// Linear velocity of the wheels m/minutes
		vel_target_right_minute = (vel_target_right_linear * 60);
		vel_target_left_minute = (vel_target_left_linear * 60);

		// Revolution per minute of the wheels
		vel_target_right_rpm =  vel_target_right_minute / (M_PI * 0.136);
		vel_target_left_rpm =  vel_target_left_minute / (M_PI * 0.136);

		
		Kinematics::rpm rpm;
		
    //calculate for the target motor RPM and direction
    //front-left motor
    rpm.motorL = vel_target_left_rpm;
    rpm.motorL = constrain(rpm.motorL, -max_rpm_, max_rpm_);

    //front-right motor
    rpm.motorR = vel_target_right_rpm;
    rpm.motorR = constrain(rpm.motorR, -max_rpm_, max_rpm_);

    return rpm;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    Kinematics::rpm rpm;

    rpm = calculateRPM(linear_x, 0.0 , angular_z);

    return rpm;
}

Kinematics::velocities Kinematics::getVelocities(float steering_angle, int rpm1, int rpm2)
{
    Kinematics::velocities vel;
    float average_rps_x;

    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2) / 2) / 60; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    vel.linear_y = 0.0;

    //http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
    vel.angular_z =  (vel.linear_x * tan(steering_angle)) / wheels_x_distance_;

    return vel;
}

Kinematics::velocities Kinematics::getVelocities(int rpm1, int rpm2)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_a;

    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2) / 2) / 60; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s


    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2) / 2) / 60;
    vel.angular_z =  (average_rps_a * wheel_circumference_) / ((wheels_x_distance_ / 2)); //  rad/s

    return vel;
}
