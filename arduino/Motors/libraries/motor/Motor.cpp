#include "Motor.h"

Controller::Controller(int pwm_pin, int dir_pin):
    pwm_pin_(pwm_pin),
    dir_pin_(dir_pin)
{
	pinMode(pwm_pin_, OUTPUT);
	pinMode(dir_pin_, OUTPUT);
	
	// starting state
	analogWrite(pwm_pin_, abs(0));
}

void Controller::spin(int pwm)
{
	if(pwm > 0)
	{
		  digitalWrite(dir_pin_, HIGH);
	}
	else if(pwm < 0)
	{
		  digitalWrite(dir_pin_, LOW);
	}
	analogWrite(pwm_pin_, abs(pwm));
}
