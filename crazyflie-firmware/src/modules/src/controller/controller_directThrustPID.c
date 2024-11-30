#include "controller_directThrustPID.h"
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
#include "controller_customPID.h"
#include <stdbool.h>
#define constrain(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

static CustomDirectThrust_t g_self = {
    // Roll PID
    .kp_roll = 2e-4f, .ki_roll = 0.0f, .kd_roll = 1e-4f,
    .integral_max_roll = 0.0f, .output_max_roll = 100.0f,

    // Pitch PID
    .kp_pitch = 2e-4f, .ki_pitch = 0.0f, .kd_pitch = 1e-4f,
    .integral_max_pitch = 0.0f, .output_max_pitch = 100.0f,

    // Yaw PID
    .kp_yaw = 0.0f, .ki_yaw = 0.0f, .kd_yaw = 0.0f,
    .integral_max_yaw = 0.0f, .output_max_yaw = 100.0f,

    // Thrust PID
    .kp_thrust = 0.06f, .ki_thrust = 0.0f, .kd_thrust = 0.3f,
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
void map_setpoint_to_desired_direct_thrust(const setpoint_t *setpoint, desired_state_t_direct_thrust *desired_state) {
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
    float dt = 1.0f / ATTITUDE_RATE;

    desired_state_t_direct_thrust desired_state;
    current_state_t current_state;

    map_setpoint_to_desired_direct_thrust(setpoint, &desired_state);
    map_state_to_actual_direct_thrust(state, &current_state);

    if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
        // Compute errors
        float x_error = desired_state.x - (current_state.x);
        float y_error = desired_state.y - (current_state.y);
        float z_error = desired_state.z - (current_state.z);

        float roll_target = DirectThrustPID_update(&x_controller, x_error, dt, 0.0f, false);
        float pitch_target = DirectThrustPID_update(&y_controller, y_error, dt, 0.0f, false);

        roll_target = 0.0f;
        pitch_target = 0.0f;

        float roll_error = roll_target - (current_state.roll);
        float pitch_error = pitch_target - (current_state.pitch);

        float thrust = DirectThrustPID_update(&z_controller, z_error, dt, 0.27f, false);
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
    }

        // Log the outputs
    if (currentTime - lastPrintTime >= pdMS_TO_TICKS(500)) {
        DEBUG_PRINT("----------------------------------------------------------------------");
        DEBUG_PRINT("control->thrust: %f\n", (double)control->thrustSi);
        DEBUG_PRINT("goal height: %f\n", (double)desired_state.z);
        DEBUG_PRINT("current height: %f\n", (double)current_state.z);
        lastPrintTime = currentTime;
    }
}




#ifdef CRAZYFLIE_FW

#include "param.h"
#include "log.h"

/**
 * @brief Initialize the CustomDirectThrust controller firmware
 */
void controllerCustomDirectThrustFirmwareInit(void) {
    // Initialize the controller using the dynamic values in g_self
    controllerCustomPidDirectThrustInit();
}

/**
 * @brief Test the CustomDirectThrust controller firmware
 * @return True if successful
 */
bool controllerCustomDirectThrustFirmwareTest(void) {
    return controllerCustomPidDirectThrustTest();
}

/**
 * @brief CustomDirectThrust controller firmware function
 */
void controllerCustomDirectThrustFirmware(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep) 
{
    // Run the main CustomDirectThrust controller logic
    controllerCustomPidDirectThrust(control, setpoint, sensors, state, stabilizerStep);
}

// /**
//  * Tuning variables for the CustomDirectThrust Controller
//  */
// PARAM_GROUP_START(ctrlCustomDirectThrust)

// /**
//  * @brief Roll P-gain
//  */
// PARAM_ADD_CORE(PARAM_FLOAT, kp_roll, &g_self.kp_roll)
// // /**
//  * @brief Roll I-gain
//  */
// PARAM_ADD(PARAM_FLOAT, ki_roll, &g_self.ki_roll)
// /**
//  * @brief Roll D-gain
//  */
// PARAM_ADD(PARAM_FLOAT, kd_roll, &g_self.kd_roll)
// /**
//  * @brief Roll maximum accumulated error
//  */
// PARAM_ADD(PARAM_FLOAT, integral_max_roll, &g_self.integral_max_roll)
// /**
//  * @brief Roll output limit
//  */
// PARAM_ADD(PARAM_FLOAT, output_max_roll, &g_self.output_max_roll)

// /**
//  * @brief Pitch P-gain
//  */
// PARAM_ADD(PARAM_FLOAT, kp_pitch, &g_self.kp_pitch)
// /**
//  * @brief Pitch I-gain
//  */
// PARAM_ADD(PARAM_FLOAT, ki_pitch, &g_self.ki_pitch)
// /**
//  * @brief Pitch D-gain
//  */
// PARAM_ADD(PARAM_FLOAT , kd_pitch, &g_self.kd_pitch)
// /**
//  * @brief Pitch maximum accumulated error
//  */
// PARAM_ADD(PARAM_FLOAT, integral_max_pitch, &g_self.integral_max_pitch)
// /**
//  * @brief Pitch output limit
//  */
// PARAM_ADD(PARAM_FLOAT, output_max_pitch, &g_self.output_max_pitch)

// /**
//  * @brief Yaw P-gain
//  */
// PARAM_ADD(PARAM_FLOAT, kp_yaw, &g_self.kp_yaw)
// /**
//  * @brief Yaw I-gain
//  */
// PARAM_ADD(PARAM_FLOAT, ki_yaw, &g_self.ki_yaw)
// /**
//  * @brief Yaw D-gain
//  */
// PARAM_ADD(PARAM_FLOAT, kd_yaw, &g_self.kd_yaw)
// /**
//  * @brief Yaw maximum accumulated error
//  */
// PARAM_ADD(PARAM_FLOAT, integral_max_yaw, &g_self.integral_max_yaw)
// /**
//  * @brief Yaw output limit
//  */
// PARAM_ADD(PARAM_FLOAT, output_max_yaw, &g_self.output_max_yaw)

// /**
//  * @brief Thrust P-gain
//  */
// PARAM_ADD(PARAM_FLOAT, kp_thrust, &g_self.kp_thrust)
// /**
//  * @brief Thrust I-gain
//  */
// PARAM_ADD(PARAM_FLOAT, ki_thrust, &g_self.ki_thrust)
// /**
//  * @brief Thrust D-gain
//  */
// PARAM_ADD(PARAM_FLOAT, kd_thrust, &g_self.kd_thrust)
// /**
//  * @brief Thrust maximum accumulated error
//  */
// PARAM_ADD(PARAM_FLOAT, integral_max_thrust, &g_self.integral_max_thrust)
// /**
//  * @brief Thrust output limit
//  */
// PARAM_ADD(PARAM_FLOAT, output_max_thrust, &g_self.output_max_thrust)

// PARAM_GROUP_STOP(ctrlCustomDirectThrust)


// /**
//  * Logging variables for the CustomDirectThrust Controller
//  */
// LOG_GROUP_START(ctrlCustomDirectThrust)
// LOG_ADD(LOG_FLOAT, directCtrl_cmd_thrust, &g_self.kp_thrust) // Commanded thrust
// LOG_ADD(LOG_FLOAT, directCtrl_cmd_roll, &g_self.kp_roll)    // Commanded roll
// LOG_ADD(LOG_FLOAT, directCtrl_cmd_pitch, &g_self.kp_pitch)  // Commanded pitch
// LOG_ADD(LOG_FLOAT, directCtrl_cmd_yaw, &g_self.kp_yaw)      // Commanded yaw

// LOG_ADD(LOG_FLOAT, directCtrl_integral_roll, &g_self.integral_max_roll)    // Integral term roll
// LOG_ADD(LOG_FLOAT, directCtrl_integral_pitch, &g_self.integral_max_pitch)  // Integral term pitch
// LOG_ADD(LOG_FLOAT, directCtrl_integral_yaw, &g_self.integral_max_yaw)      // Integral term yaw
// LOG_ADD(LOG_FLOAT, directCtrl_integral_thrust, &g_self.integral_max_thrust)// Integral term thrust

// LOG_ADD(LOG_FLOAT, directCtrl_output_roll, &g_self.output_max_roll)    // Roll output
// LOG_ADD(LOG_FLOAT, directCtrl_output_pitch, &g_self.output_max_pitch)  // Pitch output
// LOG_ADD(LOG_FLOAT, directCtrl_output_yaw, &g_self.output_max_yaw)      // Yaw output
// LOG_ADD(LOG_FLOAT, directCtrl_output_thrust, &g_self.output_max_thrust)// Thrust output
// LOG_GROUP_STOP(ctrlCustomDirectThrust)

#endif