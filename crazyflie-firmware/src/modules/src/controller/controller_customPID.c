/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Custom implementation of a PID controller by Nikolaj Hindsbo
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
#include "controller_customPID.h"
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"
// PID Variables
static float pastAltitudeError, pastPitchError, pastRollError, pastYawRateError;
static float pastVxError, pastVyError;
static float altitudeIntegrator;
// PID Gains (fixed values, no external tuning)
static const float kp_att_rp = 3.0f;
static const float kd_att_rp = 1.0f;
static const float kp_att_y = 2.0f;
static const float kp_z = 4.0f;
static const float kd_z = 1.5f;
static const float ki_z = 0.05f;
static const float kp_vel_xy = 2.5f;
static const float kd_vel_xy = 0.8f;
void controllerCustomPidInit() {
  // Initialize PID variables
  pastAltitudeError = 0.0f;
  pastPitchError = 0.0f;
  pastRollError = 0.0f;
  pastYawRateError = 0.0f;
  pastVxError = 0.0f;
  pastVyError = 0.0f;
  altitudeIntegrator = 0.0f;
  DEBUG_PRINT("Custom PID Controller Initialized\n");
}
bool controllerCustomPidTest() {
  // Always return true to indicate the controller is functional
  return true;
}
void controllerCustomPid(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t stabilizerStep) {
//   control->controlMode = controlModeLegacy;
//   float dt = 1.0f / ATTITUDE_RATE;
//   // converting types......
//   desired_state_t desired_state;
//   actual_state_t actual_state;
//   control_commands_t control_commands;
//   // Map setpoint, state, and control types to desired, actual, and control command types
//   map_setpoint_to_desired(setpoint, &desired_state);
//   map_state_to_actual(state, &actual_state);
//   map_control_to_commands(control, &control_commands);
//   // Create and initialize gains_pid_t instance with fixed PID gains
//   gains_pid_t gains_pid = {
//       .kp_att_rp = kp_att_rp,
//       .kd_att_rp = kd_att_rp,
//       .kp_att_y = kp_att_y,
//       .kd_att_y = 0.0f,  // Assuming no derivative gain for yaw if not defined, set to 0
//       .kp_vel_xy = kp_vel_xy,
//       .kd_vel_xy = kd_vel_xy,
//       .kp_z = kp_z,
//       .kd_z = kd_z,
//       .ki_z = ki_z
//   };
//   motor_power_t motorCommands;
//   if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
//     pid_attitude_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motorCommands);
//   }
//   if (RATE_DO_EXECUTE(POSITION_RATE, stabilizerStep)) {
//     pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motorCommands);
//   }
//   // TODO: convert the stuff you find into the thrust, torque x, torque y, torque z
// //   float m1 = motorCommands.m1;
// //   float m2 = motorCommands.m2;
// //   float m3 = motorCommands.m3;
// //   float m4 = motorCommands.m4;
//   resetControlIfThrustZero(control);
}
void map_setpoint_to_desired(const setpoint_t *setpoint, desired_state_t *desired_state) {
    // desired_state->roll = setpoint->attitude.roll;
    // desired_state->pitch = setpoint->attitude.pitch;
    // desired_state->yaw_rate = setpoint->attitudeRate.yaw;
    // desired_state->altitude = setpoint->position.z;
    // desired_state->vx = setpoint->velocity.x;
    // desired_state->vy = setpoint->velocity.y;
}
void map_state_to_actual(const state_t *state, actual_state_t *actual_state) {
    // actual_state->roll = state->attitude.roll;
    // actual_state->pitch = state->attitude.pitch;
    // actual_state->yaw_rate = state->attitudeRate.yaw;
    // actual_state->altitude = state->position.z;
    // actual_state->vx = state->velocity.x;
    // actual_state->vy = state->velocity.y;
}
void map_control_to_commands(const control_t *control, control_commands_t *control_commands) {
    // control_commands->roll = (double)control->roll;
    // control_commands->pitch = (double)control->pitch;
    // control_commands->yaw = (double)control->yaw;
    // control_commands->altitude = (double)control->thrust;
}
// from the github:
void pid_attitude_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                          double dt, motor_power_t *motorCommands) {
//   control_commands_t control_commands = {0};
//   pid_fixed_height_controller(actual_state, desired_state, gains_pid, dt, &control_commands);
//   pid_attitude_controller(actual_state, desired_state, gains_pid, dt, &control_commands);
//   motor_mixing(control_commands, motorCommands);
}
void pid_velocity_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                          double dt, motor_power_t *motorCommands) {
//   control_commands_t control_commands = {0};
//   pid_horizontal_velocity_controller(actual_state, desired_state, gains_pid, dt);
//   pid_fixed_height_controller(actual_state, desired_state, gains_pid, dt, &control_commands);
//   pid_attitude_controller(actual_state, desired_state, gains_pid, dt, &control_commands);
//   motor_mixing(control_commands, motorCommands);
}
void pid_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, double dt,
                                 control_commands_t *control_commands) {
//   double altitudeError = desired_state->altitude - actual_state.altitude;
//   double altitudeDerivativeError = (altitudeError - pastAltitudeError) / dt;
//   control_commands->altitude =
//     gains_pid.kp_z * constrain(altitudeError, -1, 1) + gains_pid.kd_z * altitudeDerivativeError + gains_pid.ki_z;
//   altitudeIntegrator += altitudeError * dt;
//   control_commands->altitude = gains_pid.kp_z * constrain(altitudeError, -1, 1) + gains_pid.kd_z * altitudeDerivativeError +
//                                gains_pid.ki_z * altitudeIntegrator + 48;
//   pastAltitudeError = altitudeError;
}
void motor_mixing(control_commands_t control_commands, motor_power_t *motorCommands) {
  // Motor mixing
//   motorCommands->m1 = control_commands.altitude - control_commands.roll + control_commands.pitch + control_commands.yaw;
//   motorCommands->m2 = control_commands.altitude - control_commands.roll - control_commands.pitch - control_commands.yaw;
//   motorCommands->m3 = control_commands.altitude + control_commands.roll - control_commands.pitch + control_commands.yaw;
//   motorCommands->m4 = control_commands.altitude + control_commands.roll + control_commands.pitch - control_commands.yaw;
}
void pid_attitude_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, double dt,
                             control_commands_t *control_commands) {
//   // Calculate errors
//   double pitchError = desired_state->pitch - actual_state.pitch;
//   double pitchDerivativeError = (pitchError - pastPitchError) / dt;
//   double rollError = desired_state->roll - actual_state.roll;
//   double rollDerivativeError = (rollError - pastRollError) / dt;
//   double yawRateError = desired_state->yaw_rate - actual_state.yaw_rate;
//   // PID control
//   control_commands->roll = gains_pid.kp_att_rp * constrain(rollError, -1, 1) + gains_pid.kd_att_rp * rollDerivativeError;
//   control_commands->pitch = -gains_pid.kp_att_rp * constrain(pitchError, -1, 1) - gains_pid.kd_att_rp * pitchDerivativeError;
//   control_commands->yaw = gains_pid.kp_att_y * constrain(yawRateError, -1, 1);
//   // Save error for the next round
//   pastPitchError = pitchError;
//   pastRollError = rollError;
//   pastYawRateError = yawRateError;
}
void pid_horizontal_velocity_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                        double dt) {
//   double vxError = desired_state->vx - actual_state.vx;
//   double vxDerivative = (vxError - pastVxError) / dt;
//   double vyError = desired_state->vy - actual_state.vy;
//   double vyDerivative = (vyError - pastVyError) / dt;
//   // PID control
//   double pitchCommand = gains_pid.kp_vel_xy * constrain(vxError, -1, 1) + gains_pid.kd_vel_xy * vxDerivative;
//   double rollCommand = -gains_pid.kp_vel_xy * constrain(vyError, -1, 1) - gains_pid.kd_vel_xy * vyDerivative;
//   desired_state->pitch = pitchCommand;
//   desired_state->roll = rollCommand;
//   // Save error for the next round
//   pastVxError = vxError;
//   pastVyError = vyError;
}