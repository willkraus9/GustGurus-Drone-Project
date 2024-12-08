#ifndef SLIDING_MODE_CONTROL_H
#define SLIDING_MODE_CONTROL_H

#include <math.h>

typedef struct {
    float nu = 3.3;
    float g = 9.81;
    float A = 0.35; // Drag constant
    float m = 0.028; // Mass of the quadrotor
    float alpha = 4.0; // Sliding mode control parameter
} params_t;

float calculate_sliding_manifold_z(float error_z, float error_d_z, params_t params);

float sign(float value);

float get_beta(float estimated_disturbance, float error_dz, float roll, float pitch, params_t params);

float calculate_thrust(float error_z, float error_d_z, float estimated_disturbance, float roll, float pitch, params_t params);

#endif // SLIDING_MODE_CONTROL_H