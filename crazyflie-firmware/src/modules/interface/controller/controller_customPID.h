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

#ifndef __CONTROLLER_CustomPID_H__
#define __CONTROLLER_CustomPID_H__

#include "stabilizer_types.h"

typedef struct motor_power_s {
  double m1;
  double m2;
  double m3;
  double m4;
} motor_power_t;

typedef struct control_commands_s {
  double roll;
  double pitch;
  double yaw;
  double altitude;
} control_commands_t;

typedef struct desired_state_s {
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} desired_state_t;

typedef struct actual_state_s {
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} actual_state_t;

typedef struct gains_pid_s {
  double kp_att_rp;
  double kd_att_rp;
  double kp_att_y;
  double kd_att_y;
  double kp_vel_xy;
  double kd_vel_xy;
  double kp_z;
  double kd_z;
  double ki_z;
} gains_pid_t;

void controllerCustomPidInit(void);
bool controllerCustomPidTest(void);
void controllerCustomPid(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const uint32_t stabilizerStep);
void map_setpoint_to_desired(const setpoint_t *setpoint, desired_state_t *desired_state);
void map_state_to_actual(const state_t *state, actual_state_t *actual_state);
void map_control_to_commands(const control_t *control, control_commands_t *control_commands);
void pid_attitude_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                          double dt, motor_power_t *motorCommands);
void pid_velocity_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                          double dt, motor_power_t *motorCommands);
void pid_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, double dt,
                                 control_commands_t *control_commands);
void pid_attitude_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, double dt,
                             control_commands_t *control_commands);
void pid_horizontal_velocity_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                        double dt);
void motor_mixing(control_commands_t control_commands, motor_power_t *motorCommands);
void resetControlIfThrustZero(control_t *control);

#endif //__CONTROLLER_CustomPID_H__
