#ifndef __CONTROLLER_DirectThrustPID_H__
#define __CONTROLLER_DirectThrustPID_H__
#include "stabilizer_types.h"
#include <stdbool.h>
#include "controller.h"

// Custom Direct Thrust controller type
typedef struct {
  // PID gains for roll, pitch, yaw
  float kp_roll, ki_roll, kd_roll;
  float kp_pitch, ki_pitch, kd_pitch;
  float kp_yaw, ki_yaw, kd_yaw;

  // PID gains for thrust
  float kp_thrust, ki_thrust, kd_thrust;

  // Integral term limits
  float integral_max_roll, integral_max_pitch, integral_max_yaw, integral_max_thrust;

  // Output limits
  float output_max_roll, output_max_pitch, output_max_yaw, output_max_thrust;

} CustomDirectThrust_t;

typedef struct desired_state_s_direct_thrust {
  float x;
  float y;
  float z;
  float yaw;
} desired_state_t_direct_thrust;

typedef struct current_state_s {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
} current_state_t;

typedef struct {
    float kp, ki, kd;
    float integral_max, output_max;
    float error_integral, error_previous;
} PID_t;

typedef struct {
    PID_t x_controller;
    PID_t y_controller;
    PID_t z_controller;
    PID_t roll_controller;
    PID_t pitch_controller;
    PID_t yaw_controller;
} CustomPidController_t;


// Initialize the PID structure
void DirectThrustPID_init(PID_t *pid, float kp, float ki, float kd, float integral_max, float output_max);

// Update the PID controller
float DirectThrustPID_update(PID_t *pid, float error, float dt, float baseline, bool verbose);

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

void map_setpoint_to_desired_direct_thrust(const setpoint_t *setpoint, desired_state_t_direct_thrust *desired_state);

void map_state_to_actual_direct_thrust(const state_t *state, current_state_t *current_state);

#ifdef CRAZYFLIE_FW
/** Firmware initialization */
void controllerCustomDirectThrustFirmwareInit(void);

/** Firmware test */
bool controllerCustomDirectThrustFirmwareTest(void);

/** Firmware control */
void controllerCustomDirectThrustFirmware(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
);
#endif // CRAZYFLIE_FW

#endif // CONTROLLER_CUSTOMPID_DIRECT_THRUST_H__