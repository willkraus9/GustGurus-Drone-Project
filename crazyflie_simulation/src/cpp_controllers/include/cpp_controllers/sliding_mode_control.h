#ifndef SLIDING_MODE_CONTROL_H
#define SLIDING_MODE_CONTROL_H

#include <math.h>

typedef struct {
    float nu = 3.3;
    float g = 9.81;
    float A = 0.35; // Drag constant
    float m = 0.028; // Mass of the quadrotor
    float alpha_z = 4.0; // Sliding mode control parameter
    float alpha_roll = 1.80;
    float alpha_pitch = 1.80;
    float I_xx = 16.571710E-06;
    float I_yy = 16.655602E-06;
    float I_zz = 29.261652E-06;
} params_t;

float calculate_sliding_manifold_z(float error_z, float error_d_z, params_t params);

float sign(float value);

float z_compensation(float error_d_z, float roll, float pitch, params_t params);

float smc_z(float error_z, float error_d_z, float roll, float pitch, params_t params);

float get_beta(float estimated_disturbance, float error_dz, float roll, float pitch, params_t params);

float calculate_thrust(float error_z, float error_d_z, float estimated_disturbance, float roll, float pitch, params_t params);

float calculate_sliding_manifold_roll(float error_roll, float error_d_roll, params_t params);

float roll_compensation(float v_roll, float v_yaw, params_t params);

float pitch_compensation(float v_pitch, float v_yaw, params_t params);

float calculate_sliding_manifold_pitch(float error_pitch, float error_d_pitch, params_t params);

float smc_pitch(float pitch, float yaw,float error_pitch, float error_d_pitch, params_t params);

float smc_pitch2(float roll ,float yaw, float error_roll, float error_d_roll, params_t params);

float smc_roll(float roll, float yaw, float error_roll ,float error_d_roll, params_t params);

float smc_roll2(float roll, float yaw, float error_roll ,float error_d_roll, params_t params);



#endif // SLIDING_MODE_CONTROL_H