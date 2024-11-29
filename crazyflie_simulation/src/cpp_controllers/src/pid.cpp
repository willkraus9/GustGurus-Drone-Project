#include "pid.h"
#include <iostream>
#include <cmath>

pid::pid(float kp, float ki, float kd, float integral_max, float output_max) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->integral_max = integral_max;
    this->output_max = output_max;
    this->error_integral = 0;
    this->error_previous = 0;
}
pid::~pid() {}
pid::pid() {}
float pid::update(float error, float dt, float baseline, bool verbose) {
    this->error_integral += error * dt;
    this->error_integral = fminf(this->error_integral, this->integral_max);
    this->error_integral = fmaxf(this->error_integral, -this->integral_max);

    float derivative = (error - this->error_previous) / dt;
    this->error_previous = error;

    float output = this->kp * error + this->ki * this->error_integral + this->kd * derivative + baseline;

    if (verbose) {
        std::cout << "Error: " << error << std::endl;
        std::cout << "Output: " << output << std::endl;
    }

    output = fminf(output, this->output_max);
    output = fmaxf(output, -this->output_max);


    return output;
}

