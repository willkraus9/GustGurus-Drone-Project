/*
// __        _____ _   _ ____  ____  ____  _____    _    _  _______ ____  ____  
// \ \      / /_ _| \ | |  _ \| __ )|  _ \| ____|  / \  | |/ / ____|  _ \/ ___| 
//  \ \ /\ / / | ||  \| | | | |  _ \| |_) |  _|   / _ \ | ' /|  _| | |_) \___ \ 
//   \ V  V /  | || |\  | |_| | |_) |  _ <| |___ / ___ \| . \| |___|  _ < ___) |
//    \_/\_/  |___|_| \_|____/|____/|_| \_\_____/_/   \_\_|\_\_____|_| \_\____/ 
//  
//  Crazyflie control firmware
//  
//  Custom implementation of a PID controller via direct thrust/torque calculations by Nikolaj Hindsbo & Denis Alpay
//  
*/

#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>

#include "stabilizer_types.h"
#include "attitude_controller.h"
#include "position_controller.h"
#include "power_distribution.h"
#include "param.h"
#include "math3d.h"
#include <string.h>
#include <stdint.h>
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"

#include "controller.h"
#include "controller_custom2.h"
#include <stdbool.h>
#define constrain(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

// Debug mode toggle
static bool DEBUG_MODE = true;
#define DEBUG_PRINT_CONDITIONAL(fmt, ...) \
    do { if (DEBUG_MODE) DEBUG_PRINT(fmt, ##__VA_ARGS__); } while (0)

// Tuned PID values based on Lee Controller
static CustomDirectThrust_t g_self = {
    // Roll PID
    .kp_roll = 0.007f, .ki_roll = 0.03f, .kd_roll = 0.00115f,
    .integral_max_roll = 0.1f, .output_max_roll = 100.0f,

    // Pitch PID
    .kp_pitch = 0.007f, .ki_pitch = 0.03f, .kd_pitch = 0.00115f,
    .integral_max_pitch = 0.1f, .output_max_pitch = 100.0f,

    // Yaw PID
    .kp_yaw = 0.008f, .ki_yaw = 0.03f, .kd_yaw = 0.002f,
    .integral_max_yaw = 0.1f, .output_max_yaw = 100.0f,

    // Thrust PID
    .kp_thrust = 7.0f, .ki_thrust = 0.0f, .kd_thrust = 4.0f,
    .integral_max_thrust = 0.2f, .output_max_thrust = 100.0f
};

// PID controllers for each axis
static PID_t x_controller, y_controller, z_controller;
static PID_t roll_controller, pitch_controller, yaw_controller;

static CustomPidController_t customPidController;

// Initialize the PID structure
void DirectThrustPID_init(PID_t *pid, float kp, float ki, float kd, float integral_max, float output_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->error_integral = 0.0f;
    pid->error_previous = 0.0f;
}

// Map the setpoint to the desired state
void map_setpoint_to_desired_direct_thrust(const setpoint_t *setpoint, desired_state_t *desired_state) {
    desired_state->x = setpoint->position.x;
    desired_state->y = setpoint->position.y;
    desired_state->z = setpoint->position.z;
    desired_state->yaw = setpoint->attitude.yaw;
}

// Map the current state
void map_state_to_actual_direct_thrust(const state_t *state, current_state_t *current_state) {
    current_state->roll = state->attitude.roll / 57.3f;
    current_state->pitch = state->attitude.pitch / 57.3f;
    current_state->yaw = state->attitude.yaw / 57.3f;
    current_state->z = state->position.z;
    current_state->x = state->position.x;
    current_state->y = state->position.y;
}

// Update the PID controller
float DirectThrustPID_update(PID_t *pid, float error, float dt, float baseline, bool verbose) {
    pid->error_integral += error * dt;

    if (pid->error_integral > pid->integral_max) pid->error_integral = pid->integral_max;
    if (pid->error_integral < -pid->integral_max) pid->error_integral = -pid->integral_max;

    float derivative = (error - pid->error_previous) / dt;
    pid->error_previous = error;

    float output = pid->kp * error + pid->ki * pid->error_integral + pid->kd * derivative + baseline;

    if (output > pid->output_max) output = pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;

    // DEBUG_PRINT_CONDITIONAL("PID Update - Error: %.3f, Integral: %.3f, Derivative: %.3f, Output: %.3f\n",
       //                     (double)error, (double)pid->error_integral, (double)derivative, (double)output);

    return output;
}

// Reset the PID controller
void DirectThrustPID_reset(PID_t *pid) {
    pid->error_integral = 0.0f;
    pid->error_previous = 0.0f;
}

// Initialize custom PID controller
void controllerCustomPidDirectThrustInit() {
    DirectThrustPID_init(&x_controller, g_self.kp_thrust, g_self.ki_thrust, g_self.kd_thrust, g_self.integral_max_thrust, g_self.output_max_thrust);
    DirectThrustPID_init(&y_controller, g_self.kp_thrust, g_self.ki_thrust, g_self.kd_thrust, g_self.integral_max_thrust, g_self.output_max_thrust);
    DirectThrustPID_init(&z_controller, g_self.kp_thrust, g_self.ki_thrust, g_self.kd_thrust, g_self.integral_max_thrust, g_self.output_max_thrust);

    DirectThrustPID_init(&roll_controller, g_self.kp_roll, g_self.ki_roll, g_self.kd_roll, g_self.integral_max_roll, g_self.output_max_roll);
    DirectThrustPID_init(&pitch_controller, g_self.kp_pitch, g_self.ki_pitch, g_self.kd_pitch, g_self.integral_max_pitch, g_self.output_max_pitch);
    DirectThrustPID_init(&yaw_controller, g_self.kp_yaw, g_self.ki_yaw, g_self.kd_yaw, g_self.integral_max_yaw, g_self.output_max_yaw);

    customPidController.x_controller = x_controller;
    customPidController.y_controller = y_controller;
    customPidController.z_controller = z_controller;
    customPidController.roll_controller = roll_controller;
    customPidController.pitch_controller = pitch_controller;
    customPidController.yaw_controller = yaw_controller;
    DEBUG_PRINT_CONDITIONAL("NEW Custom PID Controller Initialized\n");
}

// Test the PID controller
bool controllerCustomPidDirectThrustTest() {
    return true;
}


float roll_error = 0;
float pitch_error = 0;

// Update the custom PID controller
void controllerCustomPidDirectThrust(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep)
{
    control->controlMode = controlModeForceTorque;
    static uint32_t lastPrintTime = 0;
    uint32_t currentTime = xTaskGetTickCount();
    float dt = 1.0f / RATE_MAIN_LOOP;
    float max_thrust = powerDistributionGetMaxThrust(); // N


    desired_state_t desired_state;
    current_state_t current_state;

    // Map setpoint and state to structured formats
    map_setpoint_to_desired_direct_thrust(setpoint, &desired_state);
    map_state_to_actual_direct_thrust(state, &current_state);

    if (RATE_DO_EXECUTE(RATE_MAIN_LOOP, stabilizerStep)) {
        // Compute position and velocity errors
        float x_error = desired_state.x - current_state.x;
        float y_error = desired_state.y - current_state.y;
        float z_error = desired_state.z - current_state.z;

        float x_vel_error = 0.0f; // If velocity data is available, compute errors here
        float y_vel_error = 0.0f;
        float z_vel_error = 0.0f;

        // Position control: compute roll and pitch targets
        float roll_target = DirectThrustPID_update(&x_controller, x_error + x_vel_error, dt, 0.0f, false);
        float pitch_target = DirectThrustPID_update(&y_controller, y_error + y_vel_error, dt, 0.0f, false);

        // Attitude control: compute roll and pitch errors
        roll_error = roll_target - current_state.roll;
        pitch_error = pitch_target - current_state.pitch;

        // Thrust control with velocity coupling
        float thrust = DirectThrustPID_update(&z_controller, z_error, dt, 0.27f, false)
                     + z_controller.kd * z_vel_error; // Dynamically reference z_controller

        // Torque control with cross-axis coupling
        float torqueX = DirectThrustPID_update(&roll_controller, roll_error, dt, 0.0f, false)
                      - pitch_controller.kd * pitch_error; // Dynamically reference pitch_controller
        float torqueY = DirectThrustPID_update(&pitch_controller, pitch_error, dt, 0.0f, false)
                      - roll_controller.kd * roll_error;   // Dynamically reference roll_controller

        float yaw_error = desired_state.yaw - current_state.yaw;
        float torqueZ = DirectThrustPID_update(&yaw_controller, yaw_error, dt, 0.0f, false);

        // Apply safety checks for max angles
        float max_angle = 0.50f; // Max allowable roll/pitch in radians
        if (fabsf(current_state.roll) > max_angle || fabsf(current_state.pitch) > max_angle) {
            control->thrustSi = 0.0f;
            control->torqueX = 0.0f;
            control->torqueY = 0.0f;
            control->torqueZ = 0.0f;
            DEBUG_PRINT_CONDITIONAL("Safety triggered: roll/pitch out of bounds.\n");
        } else {
            // Set control outputs
            control->thrustSi = (thrust < max_thrust) ? thrust : max_thrust;
            control->torqueX = torqueX;
            control->torqueY = torqueY;
            control->torqueZ = torqueZ;
        }

       // Combined debug logs for control outputs and errors
        if (currentTime - lastPrintTime >= pdMS_TO_TICKS(500)) {
            DEBUG_PRINT_CONDITIONAL("----------------------------------------------------------------------\n");
            DEBUG_PRINT_CONDITIONAL(
                "Control Outputs:\n ThrustSi (Sent): %.3f, Max Thrust: %.3f, Thrust (desired): %3f\n TorqueX: %.3f, TorqueY: %.3f, TorqueZ: %.3f\n",
                (double)control->thrustSi, (double)max_thrust, (double)thrust,
                (double)control->torqueX, (double)control->torqueY, (double)control->torqueZ);
            DEBUG_PRINT_CONDITIONAL(
                "Errors:\n Roll Error: %.3f, Pitch Error: %.3f, Z Error: %.3f\n",
                (double)roll_error, (double)pitch_error, (double)z_error);
            DEBUG_PRINT_CONDITIONAL(
                "State:\n Goal Height: %.3f, Current Height: %.3f\n",
                (double)desired_state.z, (double)current_state.z);
            lastPrintTime = currentTime;
        }
    }
}



#ifdef CRAZYFLIE_FW
// Firmware initialization
void controllerCustomFirmware2Init(void) {
    controllerCustomPidDirectThrustInit();
}

// Firmware test
bool controllerCustomFirmware2Test(void) {
    // always return true
    return true;
}

// Firmware control
void controllerCustomFirmware2(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
) {
    controllerCustomPidDirectThrust(control, setpoint, sensors, state, stabilizerStep);
}
#endif

#ifdef CRAZYFLIE_FW
#include "param.h"
PARAM_GROUP_START(ctrlcustom2)

// Roll PID
PARAM_ADD(PARAM_FLOAT, r_kp, &roll_controller.kp)
PARAM_ADD(PARAM_FLOAT, r_ki, &roll_controller.ki)
PARAM_ADD(PARAM_FLOAT, r_kd, &roll_controller.kd)
PARAM_ADD(PARAM_FLOAT, r_imax, &roll_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, r_omax, &roll_controller.output_max)

// Pitch PID
PARAM_ADD(PARAM_FLOAT, p_kp, &pitch_controller.kp)
PARAM_ADD(PARAM_FLOAT, p_ki, &pitch_controller.ki)
PARAM_ADD(PARAM_FLOAT, p_kd, &pitch_controller.kd)
PARAM_ADD(PARAM_FLOAT, p_imax, &pitch_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, p_omax, &pitch_controller.output_max)

// Yaw PID
PARAM_ADD(PARAM_FLOAT, y_kp, &yaw_controller.kp)
PARAM_ADD(PARAM_FLOAT, y_ki, &yaw_controller.ki)
PARAM_ADD(PARAM_FLOAT, y_kd, &yaw_controller.kd)
PARAM_ADD(PARAM_FLOAT, y_imax, &yaw_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, y_omax, &yaw_controller.output_max)

// X-axis Thrust PID
PARAM_ADD(PARAM_FLOAT, x_kp, &x_controller.kp)
PARAM_ADD(PARAM_FLOAT, x_ki, &x_controller.ki)
PARAM_ADD(PARAM_FLOAT, x_kd, &x_controller.kd)
PARAM_ADD(PARAM_FLOAT, x_imax, &x_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, x_omax, &x_controller.output_max)

// Y-axis Thrust PID
PARAM_ADD(PARAM_FLOAT, y_t_kp, &y_controller.kp)
PARAM_ADD(PARAM_FLOAT, y_t_ki, &y_controller.ki)
PARAM_ADD(PARAM_FLOAT, y_t_kd, &y_controller.kd)
PARAM_ADD(PARAM_FLOAT, y_t_imax, &y_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, y_t_omax, &y_controller.output_max)

// Z-axis Thrust PID
PARAM_ADD(PARAM_FLOAT, z_kp, &z_controller.kp)
PARAM_ADD(PARAM_FLOAT, z_ki, &z_controller.ki)
PARAM_ADD(PARAM_FLOAT, z_kd, &z_controller.kd)
PARAM_ADD(PARAM_FLOAT, z_imax, &z_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, z_omax, &z_controller.output_max)

PARAM_GROUP_STOP(ctrlcustom2)


LOG_GROUP_START(ctrlCustom2)

// LOG_ADD(LOG_FLOAT, KR_x, &g_self.KR.x)
// LOG_ADD(LOG_FLOAT, KR_y, &g_self.KR.y)
// LOG_ADD(LOG_FLOAT, KR_z, &g_self.KR.z)
// LOG_ADD(LOG_FLOAT, Kw_x, &g_self.Komega.x)
// LOG_ADD(LOG_FLOAT, Kw_y, &g_self.Komega.y)
// LOG_ADD(LOG_FLOAT, Kw_z, &g_self.Komega.z)

// LOG_ADD(LOG_FLOAT,Kpos_Px, &g_self.Kpos_P.x)
// LOG_ADD(LOG_FLOAT,Kpos_Py, &g_self.Kpos_P.y)
// LOG_ADD(LOG_FLOAT,Kpos_Pz, &g_self.Kpos_P.z)
// LOG_ADD(LOG_FLOAT,Kpos_Dx, &g_self.Kpos_D.x)
// LOG_ADD(LOG_FLOAT,Kpos_Dy, &g_self.Kpos_D.y)
// LOG_ADD(LOG_FLOAT,Kpos_Dz, &g_self.Kpos_D.z)


// LOG_ADD(LOG_FLOAT, thrustSi, &g_self.thrustSi)
// LOG_ADD(LOG_FLOAT, torquex, &g_self.u.x)
// LOG_ADD(LOG_FLOAT, torquey, &g_self.u.y)
// LOG_ADD(LOG_FLOAT, torquez, &g_self.u.z)

// // current angles
// LOG_ADD(LOG_FLOAT, rpyx, &g_self.rpy.x)
// LOG_ADD(LOG_FLOAT, rpyy, &g_self.rpy.y)
// LOG_ADD(LOG_FLOAT, rpyz, &g_self.rpy.z)

// // desired angles
// LOG_ADD(LOG_FLOAT, rpydx, &g_self.rpy_des.x)
// LOG_ADD(LOG_FLOAT, rpydy, &g_self.rpy_des.y)
// LOG_ADD(LOG_FLOAT, rpydz, &g_self.rpy_des.z)

// // errors
// LOG_ADD(LOG_FLOAT, error_posx, &g_self.p_error.x)
// LOG_ADD(LOG_FLOAT, error_posy, &g_self.p_error.y)
// LOG_ADD(LOG_FLOAT, error_posz, &g_self.p_error.z)

// LOG_ADD(LOG_FLOAT, error_velx, &g_self.v_error.x)
// LOG_ADD(LOG_FLOAT, error_vely, &g_self.v_error.y)
// LOG_ADD(LOG_FLOAT, error_velz, &g_self.v_error.z)

// // omega
// LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
// LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
// LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

// // omega_r
// LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
// LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
// LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

// // LOG_ADD(LOG_UINT32, ticks, &ticks)

LOG_GROUP_STOP(ctrlCustom2)

#endif // CRAZYFLIE_FW defined
