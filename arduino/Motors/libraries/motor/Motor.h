#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Controller
{
    public:
        Controller(int pwm_pin, int dir_pin);
        void spin(int pwm);

    private:
        int pwm_pin_;
        int dir_pin_;
};

#endif
