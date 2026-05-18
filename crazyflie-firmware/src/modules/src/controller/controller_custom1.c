/*
// __        _____ _   _ ____  ____  ____  _____    _    _  _______ ____  ____  
// \ \      / /_ _| \ | |  _ \| __ )|  _ \| ____|  / \  | |/ / ____|  _ \/ ___| 
//  \ \ /\ / / | ||  \| | | | |  _ \| |_) |  _|   / _ \ | ' /|  _| | |_) \___ \ 
//   \ V  V /  | || |\  | |_| | |_) |  _ <| |___ / ___ \| . \| |___|  _ < ___) |
//    \_/\_/  |___|_| \_|____/|____/|_| \_\_____/_/   \_\_|\_\_____|_| \_\____/ 
//  
//  Crazyflie control firmware
//  
//  Custom implementation of a PID controller via simulation (motor speed control) by Nikolaj Hindsbo & Denis Alpay
//  
*/


#include "stabilizer_types.h"
#include "attitude_controller.h"
#include "position_controller.h"
#include "controller.h"
#include "log.h"
#include "param.h"
#include "math3d.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "controller_custom1.h"
#include "controller_custom2.h"
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"
#define constrain(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

static PID_t x_controller = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .output_max = 0.2f, .output_min = -0.2f};
static PID_t y_controller = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .output_max = 0.2f, .output_min = -0.2f};
static PID_t z_controller = {.kp = 0.1f, .ki = 0.0f, .kd = 0.05f, .integral_max = 0.0f, .output_max = 0.4f, .output_min = 0.25f};
static PID_t yaw_controller = {.kp = 0.001f, .ki = 0.0f, .kd = 3e-5f, .integral_max = 0.0f, .output_max = 1.0f, .output_min = -1.0f};
static PID_t roll_controller = {.kp = 0.003f, .ki = 1e-3f, .kd = 0.0008f, .integral_max = 8e-4f, .output_max = 1.0f, .output_min = -1.0f};
static PID_t pitch_controller = {.kp = 0.003f, .ki = 1e-3f, .kd = 0.0008f, .integral_max = 8e-4f, .output_max = 1.0f, .output_min = -1.0f};

float roll_const = 1.0;
float pitch_const = 1.0;
SMC1_params_t params = {
    .nu = 3.3,
    .g = 9.81,
    .A = 0.35, // Drag constant
    .m = 0.031, // Mass of the quadrotor
    .alpha_z = 25, // Sliding mode control parameter original 4.0
    .alpha_roll = 0.8,
    .alpha_pitch = 0.8,
    .I_xx = 16.571710E-06,
    .I_yy = 16.655602E-06,
    .I_zz = 29.261652E-06,
    .gain_z = 1,
    .gain_roll = 0.13,
    .gain_pitch = 0.19,
    .gain_yaw = 1,
    .roll_k1 = 0.003,
    .roll_k2 = 0.00001,
    .pitch_k1 = 0.003,
    .pitch_k2 = 0.00001,
    .roll_eps = 0.0,
    .pitch_eps = 0.0,
};

float calculate_sliding_manifold_z(float error_z, float error_d_z, SMC1_params_t params) {
    float alpha = params.alpha_z;
    float sliding_manifold = error_d_z + alpha * error_z;

    return sliding_manifold;
}

float sign(float value) {
    if (value > 0) return 1.0;
    if (value < 0) return -1.0;
    return 0.0;
}

float z_compensation(float error_d_z, float roll, float pitch, SMC1_params_t params) {
    float alpha = params.alpha_z;
    float g = params.g;
    float A = params.A;
    float m = params.m;

    float numerator = m*(g + (A/m + alpha)*error_d_z);
    float denominator = cos(roll)*cos(pitch);
    float z_comp = numerator/denominator;

    return z_comp;
}

float roll_compensation(float d_e_roll, float d_pitch, float d_yaw, SMC1_params_t params) {
    float I_xx = params.I_xx;
    float I_yy = params.I_yy;
    float I_zz = params.I_zz;
    float alpha = params.alpha_roll;

    float comp = I_xx*((I_yy - I_zz)/I_xx * d_pitch * d_yaw + alpha * d_e_roll);

    return comp;
}

float pitch_compensation(float d_e_pitch, float d_roll, float d_yaw, SMC1_params_t params) {
    float I_xx = params.I_xx;
    float I_yy = params.I_yy;
    float I_zz = params.I_zz;
    float alpha = params.alpha_pitch;

    float comp = I_yy*((I_zz - I_xx)/I_yy * d_roll * d_yaw + alpha * d_e_pitch);

    return comp;
}

float get_beta(float estimated_disturbance, float error_dz, float roll, float pitch, SMC1_params_t params)
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

float calculate_thrust(float error_z, float error_d_z, float estimated_disturbance, float roll, float pitch, SMC1_params_t params) {
    float sliding_manifold = calculate_sliding_manifold_z(error_z, error_d_z, params);
    float beta = get_beta(estimated_disturbance, error_d_z, roll, pitch, params);
    return sign(sliding_manifold) * beta;
}

float smc_z(float error_z, float error_d_z, float roll, float pitch, SMC1_params_t params)
{
    float z_comp = z_compensation(error_d_z, roll, pitch, params);
    float sliding_manifold = calculate_sliding_manifold_z(error_z, error_d_z, params);


    float output = params.gain_z*(error_z * error_z) * sign(sliding_manifold) + z_comp;

    return output;
}

float calculate_sliding_manifold_pitch(float error_pitch, float error_d_pitch, SMC1_params_t params) {
    float alpha = params.alpha_pitch;
    float sliding_manifold = error_d_pitch + alpha * error_pitch;

    return sliding_manifold;
}

float calculate_sliding_manifold_roll(float error_roll, float error_d_roll, SMC1_params_t params) {
    float alpha = params.alpha_roll;
    float sliding_manifold = error_d_roll + alpha * error_roll;

    return sliding_manifold;
}

float smc_pitch(float error_pitch, float d_roll, float d_yaw, float error_d_pitch, SMC1_params_t params)
{
    float sliding_manifold = calculate_sliding_manifold_pitch(error_pitch, error_d_pitch, params);
    float pitch_comp = pitch_compensation(error_d_pitch, d_roll, d_yaw, params);

    float output = params.gain_pitch*(error_pitch * error_pitch) * sign(sliding_manifold) - pitch_comp;

    return output;
}

float smc_pitch2(float error_pitch, float d_roll, float d_yaw, float error_d_pitch, SMC1_params_t params)
{
    float sliding_manifold = calculate_sliding_manifold_pitch(error_pitch, error_d_pitch, params);
    float pitch_comp = pitch_compensation(error_d_pitch, d_roll, d_yaw, params);
    float k1=params.pitch_k1;
    float k2=params.pitch_k2;
    float eps = params.pitch_eps;
    float output = k1*sliding_manifold + k2/(sign(sliding_manifold)+eps) - pitch_comp;

    return output;
}

float smc_roll(float error_roll, float d_pitch, float d_yaw, float error_d_roll, SMC1_params_t params)
{
    float sliding_manifold = calculate_sliding_manifold_roll(error_roll, error_d_roll, params);
    float roll_comp = roll_compensation(error_d_roll, d_pitch, d_yaw, params);

    float output = params.gain_roll*(error_roll * error_roll) * sign(sliding_manifold) - roll_comp;

    return output;
}

float smc_roll2(float error_roll, float d_pitch, float d_yaw, float error_d_roll, SMC1_params_t params)
{
    float sliding_manifold = calculate_sliding_manifold_roll(error_roll, error_d_roll, params);
    float roll_comp = roll_compensation(error_d_roll, d_pitch, d_yaw, params);
    float k1=params.roll_k1;
    float k2=params.roll_k2;
    float eps = params.roll_eps;
    float output = k1*sliding_manifold + k2/(sign(sliding_manifold) + eps) - roll_comp;

    return output;
}


// Firmware initialization
void controllerCustomFirmware1Init(void) {
    DEBUG_PRINT("SMC controller initilaized");
}

// Firmware test
bool controllerCustomFirmware1Test(void) {
    return true;
}

// Firmware control
void controllerCustomFirmware1(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
) 
{

    control->controlMode = controlModeForceTorque;

    static uint32_t lastPrintTime = 0;
    static uint32_t lastPrintTime2 = 0;
    uint32_t currentTime = xTaskGetTickCount();

    float dt = 1.0f / ATTITUDE_RATE;
    if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
      return;
    }

    current_state_t current_state;
    desired_state_t desired_state;
    map_setpoint_to_desired_SMC(setpoint, &desired_state);
    map_state_to_actual_SMC(state, sensors, &current_state);

    bool isToppled = current_state.roll > 1.0f || current_state.roll < -1.0f || current_state.pitch > 1.0f || current_state.pitch < -1.0f;


    float error_x = desired_state.x - current_state.x;
    float error_y = desired_state.y - current_state.y;
    float error_z = desired_state.z - current_state.z;

    struct vec error = {error_x, error_y, error_z};
    struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
    struct mat33 R = quat2rotmat(q);

    struct vec bodyError = mvmul(R, error);

    float x_error_body = bodyError.x;
    float y_error_body = bodyError.y;
    float z_error_body = bodyError.z;
    
    float pitch_target = DirectThrustPID_update(&x_controller, x_error_body, dt, 0.0, false);
    float roll_target = DirectThrustPID_update(&y_controller, -y_error_body, dt, 0.0, false);

    float error_d_z = 0.0 - current_state.vz;
    float error_roll = roll_target - current_state.roll;
    float error_d_roll = 0.0 - current_state.vroll;
    float error_pitch = pitch_target - current_state.pitch;
    float error_d_pitch = 0.0 - current_state.vpitch;
    float error_yaw = desired_state.yaw - current_state.yaw;    

    error_roll = roll_const * error_roll;
    error_pitch = pitch_const * error_pitch;

    float thrust = smc_z(error_z, error_d_z, current_state.roll, current_state.pitch, params);
    float roll = smc_roll2(error_roll, current_state.vpitch, current_state.vyaw, error_d_roll, params);
    float pitch = smc_pitch2(error_pitch, current_state.vroll, current_state.vyaw, error_d_pitch, params);
    float yaw = DirectThrustPID_update(&yaw_controller, error_yaw, dt, 0.0, false);

    control->thrustSi = thrust;
    control->torqueX = roll;
    control->torqueY = pitch;   
    control->torqueZ = yaw;

    if (desired_state.z < 0.1f && current_state.z < 0.3f) {
        control->thrustSi = 0.0f;
        control->torqueX = 0.0f;
        control->torqueY = 0.0f;
        control->torqueZ = 0.0f;
    }
    

    if (currentTime - lastPrintTime >= pdMS_TO_TICKS(1000) && !isToppled && desired_state.z > 0.1f) {
            DEBUG_PRINT("----------------------------------------------------------------------\n");
            DEBUG_PRINT("Body error - x: %.3f, y: %.3f, z: %.3f\n", (double)x_error_body, (double)y_error_body, (double)z_error_body);
            DEBUG_PRINT("Thrust: %.3f, TorqueX: %.6f, TorqueY: %.6f, TorqueZ: %.6f\n", (double)thrust, (double)roll, (double)pitch, (double)yaw);
            DEBUG_PRINT("Roll target: %.3f, Pitch target: %.3f\n", (double)roll_target, (double)pitch_target);
            DEBUG_PRINT("Roll, %.3f, Pitch: %.3f\n", (double)current_state.roll, (double)current_state.pitch);
            
            lastPrintTime = currentTime;
        }

}

void map_setpoint_to_desired_SMC(const setpoint_t *setpoint, desired_state_t *desired_state) {
    desired_state->x = setpoint->position.x;
    desired_state->y = setpoint->position.y;
    desired_state->z = setpoint->position.z;
    desired_state->roll = radians(setpoint->attitude.roll);
    desired_state->pitch = -radians(setpoint->attitude.pitch);
    desired_state->yaw = radians(setpoint->attitude.yaw);
    desired_state->vx = setpoint->velocity.x;
    desired_state->vy = setpoint->velocity.y;
    desired_state->vz = setpoint->velocity.z;
    desired_state->vroll = radians(setpoint->attitudeRate.roll);
    desired_state->vpitch = -radians(setpoint->attitudeRate.pitch);
    desired_state->vyaw = radians(setpoint->attitudeRate.yaw);
}

// Map the current state
void map_state_to_actual_SMC(const state_t *state, const sensorData_t *sensors, current_state_t *current_state) {
    current_state->roll = state->attitude.roll / 57.3f;
    current_state->pitch = -state->attitude.pitch / 57.3f;
    current_state->yaw = state->attitude.yaw / 57.3f;
    current_state->z = state->position.z;
    current_state->x = state->position.x;
    current_state->y = state->position.y;
    current_state->vx = state->velocity.x;
    current_state->vy = state->velocity.y;
    current_state->vz = state->velocity.z;
    current_state->vroll = sensors->gyro.x / 57.3f;
    current_state->vpitch = sensors->gyro.y / 57.3;
    current_state->vyaw = sensors->gyro.z / 57.3f;
}

PARAM_GROUP_START(ctrlSMC)
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, nu, &params.nu)
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, g, &params.g)
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, A, &params.A)
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, m, &params.m)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, alpha_z, &params.alpha_z)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, alpha_roll, &params.alpha_roll)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, alpha_pitch, &params.alpha_pitch)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, I_xx, &params.I_xx)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, I_yy, &params.I_yy)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, I_zz, &params.I_zz)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, gain_roll, &params.gain_roll)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, gain_pitch, &params.gain_pitch)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, gain_yaw, &params.gain_yaw)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, gain_z, &params.gain_z)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, roll_k1, &params.roll_k1)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, roll_k2, &params.roll_k2)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, roll_eps, &params.roll_eps)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, p_k1, &params.pitch_k1)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, p_k2, &params.pitch_k2)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pitch_eps, &params.pitch_eps)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_x, &x_controller.kp)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_y , &y_controller.kp)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omax_x, &x_controller.output_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omax_y, &y_controller.output_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omin_x, &x_controller.output_min)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omin_y, &y_controller.output_min)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, roll_inv, &roll_const)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pitch_inv, &pitch_const)

PARAM_GROUP_STOP(ctrlSMC)
