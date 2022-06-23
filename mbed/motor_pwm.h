#ifndef _MOTOR_PWM_H_
#define _MOTOR_PWM_H_

#include "mbed.h"

class MotorPwm{
public:
    MotorPwm(PinName pin_pwm);
    void setDuty(float duty);

    void setPeriod(int period_usec);

// Private members
private:
    // Motor PWM & direction signals
    PwmOut     pwm_out_;

};


#endif