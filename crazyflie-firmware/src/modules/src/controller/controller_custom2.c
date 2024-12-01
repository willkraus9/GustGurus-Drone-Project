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

// original values (tune as need be)
static CustomDirectThrust_t g_self = {
    // Roll PID
    .kp_roll = 0.04f, .ki_roll = 0.0f, .kd_roll = 0.06f,
    .integral_max_roll = 0.0f, .output_max_roll = 100.0f,

    // Pitch PID
    .kp_pitch = 0.04f, .ki_pitch = 0.0f, .kd_pitch = 0.06f,
    .integral_max_pitch = 0.0f, .output_max_pitch = 100.0f,

    // Yaw PID
    .kp_yaw = 0.0f, .ki_yaw = 0.0f, .kd_yaw = 0.0f,
    .integral_max_yaw = 0.0f, .output_max_yaw = 100.0f,

    // Thrust PID
    .kp_thrust = 0.06f, .ki_thrust = 0.0f, .kd_thrust = 0.0f,
    .integral_max_thrust = 0.0f, .output_max_thrust = 100.0f
};

// PID controllers for each axis
static PID_t x_controller, y_controller, z_controller;
static PID_t roll_controller, pitch_controller, yaw_controller;

static CustomPidController_t customPidController;


// Initialize the PID structure
void DirectThrustPID_init(PID_t *pid, float kp, float ki, float kd, float integral_max, float output_max) {
  /*
    * Initialize the PID controller
    * @param pid: Pointer to the PID controller structure
    * @param kp: Proportional gain
    * @param ki: Integral gain
    * @param kd: Derivative gain
    * @param integral_max: Maximum value of the integral term
    * @param output_max: Maximum value of the output
  */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->error_integral = 0.0f;
    pid->error_previous = 0.0f;
}

// Update the PID controller
float DirectThrustPID_update(PID_t *pid, float error, float dt, float baseline, bool verbose) {
    // Integrate the error
    pid->error_integral += error * dt;

    // Constrain the integral term
    if (pid->error_integral > pid->integral_max) pid->error_integral = pid->integral_max;
    if (pid->error_integral < -pid->integral_max) pid->error_integral = -pid->integral_max;

    // Compute the derivative
    float derivative = (error - pid->error_previous) / dt;
    pid->error_previous = error;

    // Compute the PID output
    float output = pid->kp * error + pid->ki * pid->error_integral + pid->kd * derivative + baseline;

    // Constrain the output
    if (output > pid->output_max) output = pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;

    // DEBUG_PRINT("PID Update - Error: %.3f, Integral: %.3f, Derivative: %.3f, Output: %.3f\n",
    //             (double)error, (double)pid->error_integral, (double)derivative, (double)output);


    return output;
}

// Reset the PID controller
void DirectThrustPID_reset(PID_t *pid) {
    pid->error_integral = 0.0f;
    pid->error_previous = 0.0f;
}

// Initialize custom PID controller
void controllerCustomPidDirectThrustInit() {
    // Initialize all PID controllers
    // KP, KI, KD, Integral Max, Output Max

    // pitch is forward

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
    DEBUG_PRINT("NEW Custom PID Controller Initialized\n");
}

// Test the PID controller
bool controllerCustomPidDirectThrustTest() {
    return true; // Always return true to indicate the controller is functional
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
    static uint32_t lastPrintTime2 = 0;
    uint32_t currentTime = xTaskGetTickCount();
    float dt = 1.0f / RATE_MAIN_LOOP;

    desired_state_t desired_state;
    current_state_t current_state;

    map_setpoint_to_desired_direct_thrust(setpoint, &desired_state);
    map_state_to_actual_direct_thrust(state, &current_state);

    if (RATE_DO_EXECUTE(RATE_MAIN_LOOP, stabilizerStep)) {
        // Compute errors
        float x_error = desired_state.x - (current_state.x);
        float y_error = desired_state.y - (current_state.y);
        float z_error = desired_state.z - (current_state.z);

        float roll_target = DirectThrustPID_update(&x_controller, x_error, dt, 0.0f, false);
        float pitch_target = DirectThrustPID_update(&y_controller, y_error, dt, 0.0f, false);

        roll_target = 0.0f;
        pitch_target = 0.0f;

        roll_error = roll_target - (current_state.roll);
        pitch_error = pitch_target - (current_state.pitch);

        float thrust = DirectThrustPID_update(&z_controller, z_error, dt, 0.27, false); // change back to 0.27f for min thrust
        float torqueX = DirectThrustPID_update(&roll_controller, roll_error, dt, 0.0f, false);
        float torqueY = DirectThrustPID_update(&pitch_controller, pitch_error, dt, 0.0f, false);
        float torqueZ = DirectThrustPID_update(&yaw_controller, desired_state.yaw - (current_state.yaw), dt, 0.0f, false);

        // Set the control outputs
        control->thrustSi = thrust;
        control->torqueX = torqueX;
        control->torqueY = torqueY;
        control->torqueZ = torqueZ;

        // Safety checks
        if ((current_state.z) < 0.1f && (desired_state.z) < 0.1f) {
            control->thrustSi = 0.0f;
            control->torqueX = 0.0f;
            control->torqueY = 0.0f;
            control->torqueZ = 0.0f;
        }

        float maxdeg = 0.50f;
        if ((current_state.roll) > maxdeg || (current_state.roll) < -maxdeg ||
            (current_state.pitch) > maxdeg || (current_state.pitch) < -maxdeg) {
            control->thrustSi = 0.0f;
            control->torqueX = 0.0f;
            control->torqueY = 0.0f;
            control->torqueZ = 0.0f;
        }
        if (currentTime - lastPrintTime2 >= pdMS_TO_TICKS(500)) {
            DEBUG_PRINT("Roll error: %.3f, pitch error %.3f, torqueY %.8f \n", 
                            (double)roll_error, (double)pitch_error, (double)torqueY);
            lastPrintTime2 = currentTime;
    }
    }

    // Log the control outputs every 500ms
    // if (currentTime - lastPrintTime >= pdMS_TO_TICKS(500)) {
    //     DEBUG_PRINT("----------------------------------------------------------------------");
    //     DEBUG_PRINT("control->thrust: %f\n", (double)control->thrustSi);
    //     DEBUG_PRINT("goal height: %f\n", (double)desired_state.z);
    //     DEBUG_PRINT("current height: %f\n", (double)current_state.z);
    //     lastPrintTime = currentTime;
    // }

    // Print all controller parameters every 5 seconds

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
PARAM_GROUP_START(ctrlCustom2)

// Roll PID
PARAM_ADD(PARAM_FLOAT, kp_r, &roll_controller.kp)
PARAM_ADD(PARAM_FLOAT, ki_r, &roll_controller.ki)
PARAM_ADD(PARAM_FLOAT, kd_r, &roll_controller.kd)
PARAM_ADD(PARAM_FLOAT, imax_r, &roll_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, omax_r, &roll_controller.output_max)

// Pitch PID
PARAM_ADD(PARAM_FLOAT, kp_p, &pitch_controller.kp)
PARAM_ADD(PARAM_FLOAT, ki_p, &pitch_controller.ki)
PARAM_ADD(PARAM_FLOAT, kd_p, &pitch_controller.kd)
PARAM_ADD(PARAM_FLOAT, imax_p, &pitch_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, omax_p, &pitch_controller.output_max)

// Yaw PID
PARAM_ADD(PARAM_FLOAT, kp_y, &yaw_controller.kp)
PARAM_ADD(PARAM_FLOAT, ki_y, &yaw_controller.ki)
PARAM_ADD(PARAM_FLOAT, kd_y, &yaw_controller.kd)
PARAM_ADD(PARAM_FLOAT, imax_y, &yaw_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, omax_y, &yaw_controller.output_max)

// X-axis Thrust PID
PARAM_ADD(PARAM_FLOAT, kp_x, &x_controller.kp)
PARAM_ADD(PARAM_FLOAT, ki_x, &x_controller.ki)
PARAM_ADD(PARAM_FLOAT, kd_x, &x_controller.kd)
PARAM_ADD(PARAM_FLOAT, imax_x, &x_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, omax_x, &x_controller.output_max)

// Y-axis Thrust PID
PARAM_ADD(PARAM_FLOAT, kp_y_t, &y_controller.kp)
PARAM_ADD(PARAM_FLOAT, ki_y_t, &y_controller.ki)
PARAM_ADD(PARAM_FLOAT, kd_y_t, &y_controller.kd)
PARAM_ADD(PARAM_FLOAT, imax_y_t, &y_controller.integral_max)
PARAM_ADD(PARAM_FLOAT, omax_y_t, &y_controller.output_max)

// Z-axis Thrust PID
PARAM_ADD(PARAM_FLOAT, kp_z, &z_controller.kp)
PARAM_ADD(PARAM_FLOAT, ki_z, &z_controller.ki)
PARAM_ADD(PARAM_FLOAT, kd_z, &z_controller.kd)
PARAM_ADD(PARAM_FLOAT, imax_z, &z_controller.integral_max)


PARAM_GROUP_STOP(ctrlCustom2)


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