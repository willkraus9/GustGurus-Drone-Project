#ifndef __CONTROLLER_Custom1_H__
#define __CONTROLLER_Custom1_H__
#include "stabilizer_types.h"
#include "controller_custom_types.h"

// Initialize the custom controller



float calculate_sliding_manifold_z(float error_z, float error_d_z, SMC1_params_t params);

float sign(float value);

float z_compensation(float error_d_z, float roll, float pitch, SMC1_params_t params);

float smc_z(float error_z, float error_d_z, float roll, float pitch, SMC1_params_t params);

float get_beta(float estimated_disturbance, float error_dz, float roll, float pitch, SMC1_params_t params);

float calculate_thrust(float error_z, float error_d_z, float estimated_disturbance, float roll, float pitch, SMC1_params_t params);

float calculate_sliding_manifold_roll(float error_roll, float error_d_roll, SMC1_params_t params);

float roll_compensation(float d_e_roll, float d_pitch, float d_yaw, SMC1_params_t params);

float pitch_compensation(float d_e_pitch, float d_roll, float d_yaw, SMC1_params_t params);

float calculate_sliding_manifold_pitch(float error_pitch, float error_d_pitch, SMC1_params_t params);

float smc_pitch(float error_pitch, float d_roll, float d_yaw, float error_d_pitch, SMC1_params_t params);

float smc_pitch2(float error_pitch, float d_roll, float d_yaw, float error_d_pitch, SMC1_params_t params);

float smc_roll(float error_roll, float d_pitch, float d_yaw, float error_d_roll, SMC1_params_t params);

float smc_roll2(float error_roll, float d_pitch, float d_yaw, float error_d_roll, SMC1_params_t params);

// Firmware initialization
void controllerCustomFirmware1Init(void);

// Firmware test
bool controllerCustomFirmware1Test(void);

// Firmware control
void controllerCustomFirmware1(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
);

#endif // __CONTROLLER_CUSTOM1_H__

