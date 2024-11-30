/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ /_  / / _ \
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
#define constrain(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

// Constants for thrust and torque calculation
#define kT 1.28e-8f    // Thrust coefficient (N/(rad/s)^2)
#define gamma 0.0059f // Rotor drag-to-thrust ratio
#define L 0.03252691193458118612243884065682f      // Distance between motor and center of mass (in meters)

// PID Variables
static float pastAltitudeError, pastPitchError, pastRollError, pastYawRateError;
static float pastVxError, pastVyError;
static float altitudeIntegrator;

// PID Gains (fixed values, no external tuning)
static const float kp_att_rp = 200.0f;
static const float kd_att_rp = 100.0f;
static const float kp_att_y = 200.0f;
static const float kd_att_y = 200.0f;
static const float kp_z = 2300.0f;
static const float kd_z = 1300.0f;
static const float ki_z = 0.0f;
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
  static uint32_t lastPrintTime = 0;  // Store the last time the print was executed
  uint32_t currentTime = xTaskGetTickCount();  // Get the current tick count

  control->controlMode = controlModeForceTorque;
  desired_state_t desired_state;
  actual_state_t actual_state;
  control_commands_t control_commands;

  map_setpoint_to_desired(setpoint, &desired_state);
  map_state_to_actual(state, &actual_state);
  map_control_to_commands(control, &control_commands);

  gains_pid_t gains_pid = {
      .kp_att_rp = kp_att_rp,
      .kd_att_rp = kd_att_rp,
      .kp_att_y = kp_att_y,
      .kd_att_y = kd_att_y,
      .kp_vel_xy = kp_vel_xy,
      .kd_vel_xy = kd_vel_xy,
      .kp_z = kp_z,
      .kd_z = kd_z,
      .ki_z = ki_z
  };

  motor_power_t motorCommands;
  // Attitude rate control
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    float dt = 1.0f / ATTITUDE_RATE;

    pid_attitude_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motorCommands);
  }

  // Position rate control
  // if (RATE_DO_EXECUTE(POSITION_RATE, stabilizerStep)) {
  //   float dt = 1.0f / POSITION_RATE;

  //   pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motorCommands);
  // }

  // updaing thrusts and torques
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)){
    // Calculate thrust and torques
    float T, Tx, Ty, Tz;
    calculate_thrust_torques(&motorCommands, &T, &Tx, &Ty, &Tz);

    // Log thrust, torques, and motor commands for debugging every 1 second
    if (currentTime - lastPrintTime >= pdMS_TO_TICKS(500)) {  // Check if 1 second has passed
        DEBUG_PRINT("Thrust: %f, Torque X: %f, Torque Y: %f, Torque Z: %f\n", 
                    (double)T, (double)Tx, (double)Ty, (double)Tz);

        DEBUG_PRINT("Motor Commands - m1: %f, m2: %f, m3: %f, m4: %f\n", 
                  (double)motorCommands.m1, (double)motorCommands.m2, 
                  (double)motorCommands.m3, (double)motorCommands.m4);

        DEBUG_PRINT("Actual State - Roll: %f, Pitch: %f, Yaw Rate: %f, Altitude: %f, Vx: %f, Vy: %f\n", 
            (double)actual_state.roll, (double)actual_state.pitch, (double)actual_state.yaw_rate, 
            (double)actual_state.altitude, (double)actual_state.vx, (double)actual_state.vy);

        DEBUG_PRINT("Desired State - Roll: %f, Pitch: %f, Yaw Rate: %f, Altitude: %f, Vx: %f, Vy: %f\n", 
            (double)desired_state.roll, (double)desired_state.pitch, (double)desired_state.yaw_rate, 
            (double)desired_state.altitude, (double)desired_state.vx, (double)desired_state.vy);

        lastPrintTime = currentTime;  // Update the last print time
    }

    control->thrustSi = (float)T;
    control->torqueX= Tx;
    control->torqueY= Ty;
    control->torqueZ = Tz;
  }
  
  
}

void map_setpoint_to_desired(const setpoint_t *setpoint, desired_state_t *desired_state) {
  desired_state->roll = setpoint->attitude.roll;
  desired_state->pitch = setpoint->attitude.pitch;
  desired_state->yaw_rate = setpoint->attitudeRate.yaw;
  desired_state->altitude = setpoint->position.z;
  desired_state->vx = setpoint->velocity.x;
  desired_state->vy = setpoint->velocity.y;
}

void map_state_to_actual(const state_t *state, actual_state_t *actual_state) {
  actual_state->roll = state->attitude.roll;
  actual_state->pitch = state->attitude.pitch;
  actual_state->yaw_rate = state->attitude.yaw;
  actual_state->altitude = state->position.z;
  actual_state->vx = state->velocity.x;
  actual_state->vy = state->velocity.y;
}

void map_control_to_commands(const control_t *control, control_commands_t *control_commands) {
  control_commands->roll = control->roll;
  control_commands->pitch = control->pitch;
  control_commands->yaw = control->yaw;
  control_commands->altitude = control->thrust;
}

void pid_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, float dt,
                                 control_commands_t *control_commands) {
  float altitudeError = desired_state->altitude - actual_state.altitude;
  float altitudeDerivativeError = (altitudeError - pastAltitudeError) / dt;
  altitudeIntegrator += altitudeError * dt;

  control_commands->altitude =
    gains_pid.kp_z * constrain(altitudeError, -1.0f, 1.0f) +
    gains_pid.kd_z * altitudeDerivativeError +
    gains_pid.ki_z * altitudeIntegrator + 2330.0f;

  pastAltitudeError = altitudeError;
}

void pid_attitude_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, float dt,
                             control_commands_t *control_commands) {
  float pitchError = desired_state->pitch - actual_state.pitch;
  float pitchDerivativeError = (pitchError - pastPitchError) / dt;
  float rollError = desired_state->roll - actual_state.roll;
  float rollDerivativeError = (rollError - pastRollError) / dt;
  float yawRateError = desired_state->yaw_rate - actual_state.yaw_rate;

  control_commands->roll = gains_pid.kp_att_rp * constrain(rollError, -1.0f, 1.0f) +
                           gains_pid.kd_att_rp * rollDerivativeError;
  control_commands->pitch = -gains_pid.kp_att_rp * constrain(pitchError, -1.0f, 1.0f) -
                            gains_pid.kd_att_rp * pitchDerivativeError;
  control_commands->yaw = gains_pid.kp_att_y * constrain(yawRateError, -1.0f, 1.0f);

  pastPitchError = pitchError;
  pastRollError = rollError;
  pastYawRateError = yawRateError;
}

void pid_horizontal_velocity_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                        float dt) {
  float vxError = desired_state->vx - actual_state.vx;
  float vxDerivative = (vxError - pastVxError) / dt;
  float vyError = desired_state->vy - actual_state.vy;
  float vyDerivative = (vyError - pastVyError) / dt;

  float pitchCommand = gains_pid.kp_vel_xy * constrain(vxError, -1.0f, 1.0f) +
                       gains_pid.kd_vel_xy * vxDerivative;
  float rollCommand = -gains_pid.kp_vel_xy * constrain(vyError, -1.0f, 1.0f) -
                      gains_pid.kd_vel_xy * vyDerivative;

  desired_state->pitch = pitchCommand;
  desired_state->roll = rollCommand;

  pastVxError = vxError;
  pastVyError = vyError;
}

void pid_attitude_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                          float dt, motor_power_t *motorCommands) {
  control_commands_t control_commands = {0};

  // Fixed height controller
  pid_fixed_height_controller(actual_state, desired_state, gains_pid, dt, &control_commands);

  // Attitude controller
  pid_attitude_controller(actual_state, desired_state, gains_pid, dt, &control_commands);

  // Motor mixing
  motor_mixing(control_commands, motorCommands);
}

void pid_velocity_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                          float dt, motor_power_t *motorCommands) {
  control_commands_t control_commands = {0};

  // Horizontal velocity controller
  pid_horizontal_velocity_controller(actual_state, desired_state, gains_pid, dt);

  // Fixed height controller
  pid_fixed_height_controller(actual_state, desired_state, gains_pid, dt, &control_commands);

  // Attitude controller
  pid_attitude_controller(actual_state, desired_state, gains_pid, dt, &control_commands);

  // Motor mixing
  motor_mixing(control_commands, motorCommands);
}


void motor_mixing(control_commands_t control_commands, motor_power_t *motorCommands) {
  // Motor mixing
  motorCommands->m1 = control_commands.altitude - control_commands.roll + control_commands.pitch + control_commands.yaw;
  motorCommands->m2 = control_commands.altitude - control_commands.roll - control_commands.pitch - control_commands.yaw;
  motorCommands->m3 = control_commands.altitude + control_commands.roll - control_commands.pitch + control_commands.yaw;
  motorCommands->m4 = control_commands.altitude + control_commands.roll + control_commands.pitch - control_commands.yaw;
}




void calculate_thrust_torques(const motor_power_t *motorCommands, float *T, float *Tx, float *Ty, float *Tz) {
    // Convert motor commands to forces
    float F1 = kT * motorCommands->m1 * motorCommands->m1;
    float F2 = kT * motorCommands->m2 * motorCommands->m2;
    float F3 = kT * motorCommands->m3 * motorCommands->m3;
    float F4 = kT * motorCommands->m4 * motorCommands->m4;

    // Calculate total thrust
    *T = F1 + F2 + F3 + F4;

    // Calculate torques based on motor positions and forces
    *Tx = L * (-F1 + F2 + F3 - F4);      // Torque around X-axis
    *Ty = L * (F1 + F2 - F3 - F4);       // Torque around Y-axis
    *Tz = gamma * (-F1 + F2 - F3 + F4);  // Torque around Z-axis
}