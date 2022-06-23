#include "motor_pwm.h"

MotorPwm::MotorPwm(PinName pin_pwm)
: pwm_out_(pin_pwm)
{
    // Default period = 0.0025 sec (400 Hz)
    setPeriod(2500); // us
};

void MotorPwm::setPeriod(int period_usec){
    pwm_out_.period_us(period_usec);
};


void MotorPwm::setDuty(float duty){
    if(duty > 1.0f) duty = 1.0f;
    if(duty < 0.0f) duty = 0.0f;
    // pwm_out_.write(duty);
    pwm_out_.write(duty);
};