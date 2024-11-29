#pragma once
#include <Eigen/Dense>

class pid {
    public:
        float error_integral;
        float error_previous;
        float kp;
        float ki;
        float kd;
        float output_max;
        float integral_max;

        pid(float kp, float ki, float kd, float integral_max, float output_max);
        ~pid();
        pid();
        float update(float error, float dt, float baseline, bool verbose=false);
};