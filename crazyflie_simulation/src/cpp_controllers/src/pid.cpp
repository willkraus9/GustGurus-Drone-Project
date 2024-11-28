#include "pid.h"

pid::pid(float kp, float ki, float kd, float integral_max) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->integral_max = integral_max;
    this->error_integral = 0;
    this->error_previous = 0;
}

float pid::update(float error, float dt) {
    this->error_integral += error * dt;
    this->error_integral = fminf(this->error_integral, this->integral_max);
    this->error_integral = fmaxf(this->error_integral, -this->integral_max);

    float derivative = (error - this->error_previous) / dt;
    this->error_previous = error;

    return this->kp * error + this->ki * this->error_integral + this->kd * derivative;
}

