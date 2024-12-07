#ifndef __CONTROLLER_Custom1_H__
#define __CONTROLLER_Custom1_H__
#include "stabilizer_types.h"
#include "controller_custom_types.h"

// Initialize the custom controller
void controllerCustomInit1(void);

// Test the custom controller
bool controllerCustomTest1(void);

// Main custom controller function
void controllerCustom1(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
);

// Map setpoint to desired state
void map_setpoint_to_desired(const setpoint_t *setpoint, desired_state_t *desired_state);

// Map actual state
void map_state_to_actual(const state_t *state, actual_state_t *actual_state);

// Map control to commands
void map_control_to_commands(const control_t *control, control_commands_t *control_commands);

void multiply_matrices(const float mat1[3][3], const float mat2[3][3], float result[3][3]);

// PID-based fixed height controller
void pid_fixed_height_controller(
    actual_state_t actual_state,
    desired_state_t *desired_state,
    gains_pid_t gains_pid,
    float dt,
    control_commands_t *control_commands
);

// PID-based attitude controller
void pid_attitude_controller(
    actual_state_t actual_state,
    desired_state_t *desired_state,
    gains_pid_t gains_pid,
    float dt,
    control_commands_t *control_commands
);

// PID-based horizontal velocity controller
void pid_horizontal_velocity_controller(
    actual_state_t actual_state,
    desired_state_t *desired_state,
    gains_pid_t gains_pid,
    float dt
);

// Combined PID-based attitude and fixed height controller
void pid_attitude_fixed_height_controller(
    actual_state_t actual_state,
    desired_state_t *desired_state,
    gains_pid_t gains_pid,
    float dt,
    motor_power_t *motorCommands
);

// Combined PID-based velocity and fixed height controller
void pid_velocity_fixed_height_controller(
    actual_state_t actual_state,
    desired_state_t *desired_state,
    gains_pid_t gains_pid,
    float dt,
    motor_power_t *motorCommands
);

// Motor mixing function
void motor_mixing(control_commands_t control_commands, motor_power_t *motorCommands);

// Calculate thrust and torques
void calculate_thrust_torques(
    const motor_power_t *motorCommands,
    float *T,
    float *Tx,
    float *Ty,
    float *Tz
);

#ifdef CRAZYFLIE_FW
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
#endif // CRAZYFLIE_FW

#endif // __CONTROLLER_CUSTOM1_H__
