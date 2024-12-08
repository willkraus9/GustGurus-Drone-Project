
#include <math.h>
#include "sliding_mode_control.h"
#include <iostream>

float calculate_sliding_manifold_z(float error_z, float error_d_z, params_t params) {
    float alpha = params.alpha;
    float sliding_manifold = error_d_z + alpha * error_z;

    return sliding_manifold;
}

float sign(float value) {
    if (value > 0) return 1.0;
    if (value < 0) return -1.0;
    return 0.0;
}

float get_beta(float estimated_disturbance, float error_dz, float roll, float pitch, params_t params)
{

    float alpha = params.alpha;
    float nu = params.nu;
    float g = params.g;
    float A = params.A;
    float m = params.m;

    float beta = (nu + g + ((A / m) + alpha) * error_dz) / (cos(roll) * cos(pitch));
    beta = beta * m;
    return beta;

}

float calculate_thrust(float error_z, float error_d_z, float estimated_disturbance, float roll, float pitch, params_t params) {
    float sliding_manifold = calculate_sliding_manifold_z(error_z, error_d_z, params);
    float beta = get_beta(estimated_disturbance, error_d_z, roll, pitch, params);
    std::cout << "beta: " << beta << std::endl;
    return sign(sliding_manifold) * beta;
}