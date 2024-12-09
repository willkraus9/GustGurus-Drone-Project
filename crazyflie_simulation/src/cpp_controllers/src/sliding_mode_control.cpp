
#include <math.h>
#include "sliding_mode_control.h"
#include <Eigen/Dense>
#include <iostream>

float calculate_sliding_manifold_z(float error_z, float error_d_z, params_t params) {
    float alpha = params.alpha_z;
    float sliding_manifold = error_d_z + alpha * error_z;

    return sliding_manifold;
}

float sign(float value) {
    if (value > 0) return 1.0;
    if (value < 0) return -1.0;
    return 0.0;
}

float z_compensation(float error_d_z, float roll, float pitch, params_t params) {
    float alpha = params.alpha_z;
    float g = params.g;
    float A = params.A;
    float m = params.m;

    float numerator = m*(g + (A/m + alpha)*error_d_z);
    float denominator = cos(roll)*cos(pitch);
    float z_comp = numerator/denominator;

    return z_comp;
}

float roll_compensation(float d_e_roll, float d_pitch, float d_yaw, params_t params) {
    float I_xx = params.I_xx;
    float I_yy = params.I_yy;
    float I_zz = params.I_zz;
    float alpha = params.alpha_roll;

    float comp = I_xx*((I_yy - I_zz)/I_xx * d_pitch * d_yaw + alpha * d_e_roll);

    return comp;
}

float pitch_compensation(float d_e_pitch, float d_roll, float d_yaw, params_t params) {
    float I_xx = params.I_xx;
    float I_yy = params.I_yy;
    float I_zz = params.I_zz;
    float alpha = params.alpha_pitch;

    float comp = I_yy*((I_zz - I_xx)/I_yy * d_roll * d_yaw + alpha * d_e_pitch);

    return comp;
}

float get_beta(float estimated_disturbance, float error_dz, float roll, float pitch, params_t params)
{

    float alpha = params.alpha_z;
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

float smc_z(float error_z, float error_d_z, float roll, float pitch, params_t params)
{
    float z_comp = z_compensation(error_d_z, roll, pitch, params);
    float sliding_manifold = calculate_sliding_manifold_z(error_z, error_d_z, params);


    float output = 300*(error_z * error_z) * sign(sliding_manifold) + z_comp;

    return output;
    // return output * sign(sliding_manifold);
}

float calculate_sliding_manifold_pitch(float error_pitch, float error_d_pitch, params_t params) {
    float alpha = params.alpha_pitch;
    float sliding_manifold = error_d_pitch + alpha * error_pitch;

    return sliding_manifold;
}

float calculate_sliding_manifold_roll(float error_roll, float error_d_roll, params_t params) {
    float alpha = params.alpha_roll;
    float sliding_manifold = error_d_roll + alpha * error_roll;

    return sliding_manifold;
}

float smc_pitch(float error_pitch, float d_roll, float d_yaw, float error_d_pitch, params_t params)
{
    float sliding_manifold = calculate_sliding_manifold_pitch(error_pitch, error_d_pitch, params);
    float pitch_comp = pitch_compensation(error_d_pitch, d_roll, d_yaw, params);

    float output = 0.5*(error_pitch * error_pitch) * sign(sliding_manifold) - pitch_comp;

    return output;
}

float smc_pitch2(float error_pitch, float d_roll, float d_yaw, float error_d_pitch, params_t params)
{
    float sliding_manifold = calculate_sliding_manifold_pitch(error_pitch, error_d_pitch, params);
    float pitch_comp = pitch_compensation(error_d_pitch, d_roll, d_yaw, params);
    float k1=10.0;
    float k2=14.40;
    float output = k1*sliding_manifold + k2/(sign(sliding_manifold)+0.1) - pitch_comp;

    return output;
}

float smc_roll(float error_roll, float d_pitch, float d_yaw, float error_d_roll, params_t params)
{
    float sliding_manifold = calculate_sliding_manifold_roll(error_roll, error_d_roll, params);
    float roll_comp = roll_compensation(error_d_roll, d_pitch, d_yaw, params);

    float output = 0.5*(error_roll * error_roll) * sign(sliding_manifold) - roll_comp;

    return output;
}

float smc_roll2(float error_roll, float d_pitch, float d_yaw, float error_d_roll, params_t params)
{
    float sliding_manifold = calculate_sliding_manifold_roll(error_roll, error_d_roll, params);
    float roll_comp = roll_compensation(error_d_roll, d_pitch, d_yaw, params);
    float k1=10.0;
    float k2=14.40;
    float output = k1*sliding_manifold + k2/(sign(sliding_manifold)+0.1) - roll_comp;

    return output;
}

Eigen::Vector3f World2BodyError(Eigen::Vector3f error, Eigen::Vector3f orientation)
{
    float roll = orientation(0);
    float pitch = orientation(1);
    float yaw = orientation(2);

    // Compute the rotation matrix from world to body frame using explicit matrices
    Eigen::Matrix3f rotation;

    // Roll (rotation about X-axis)
    Eigen::Matrix3f rollMatrix;
    rollMatrix << 1, 0, 0,
                  0, cos(roll), -sin(roll),
                  0, sin(roll), cos(roll);

    // Pitch (rotation about Y-axis)
    Eigen::Matrix3f pitchMatrix;
    pitchMatrix << cos(pitch), 0, sin(pitch),
                   0, 1, 0,
                   -sin(pitch), 0, cos(pitch);

    // Yaw (rotation about Z-axis)
    Eigen::Matrix3f yawMatrix;
    yawMatrix << cos(yaw), -sin(yaw), 0,
                 sin(yaw), cos(yaw), 0,
                 0, 0, 1;

    // Combine the rotations to get the world-to-body rotation matrix
    rotation = yawMatrix * pitchMatrix * rollMatrix;

    // Transform error from world frame to body frame
    return rotation.transpose() * error;
}