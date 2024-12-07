#ifndef __CONTROLLER_DIRECT_THRUST_PID_H__
#define __CONTROLLER_DIRECT_THRUST_PID_H__

#include "stabilizer_types.h"
#include "controller_custom_types.h"
#include "math.h"
// Initialize the PID structure
void DirectThrustPID_init(PID_t *pid, float kp, float ki, float kd, float integral_max, float output_max);

Vector3f multiplyMatrixVector(float matrix[3][3], Vector3f vector);

// Reset the PID controller
void DirectThrustPID_reset(PID_t *pid);

// Custom PID controller initialization
void controllerCustomPidDirectThrustInit(void);

// Custom PID controller test
bool controllerCustomPidDirectThrustTest(void);

// Custom PID controller update
void controllerCustomPidDirectThrust(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
);

// Map setpoint to desired state
void map_setpoint_to_desired_direct_thrust(const setpoint_t *setpoint, desired_state_t *desired_state);

// Map state to actual state
void map_state_to_actual_direct_thrust(const state_t *state, current_state_t *current_state);

#ifdef CRAZYFLIE_FW
/** Firmware initialization */
void controllerCustomFirmware2Init(void);

/** Firmware test */
bool controllerCustomFirmware2Test(void);

/** Firmware control */
void controllerCustomFirmware2(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
);
#endif // CRAZYFLIE_FW

#endif // __CONTROLLER_DIRECT_THRUST_PID_H__
