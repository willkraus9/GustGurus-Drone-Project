#ifndef __CONTROLLER_CustomPID_H__
#define __CONTROLLER_CustomPID_H__
#include "stabilizer_types.h"

typedef struct motor_power_s {
  float m1;
  float m2;
  float m3;
  float m4;
} motor_power_t;

typedef struct control_commands_s {
  float roll;
  float pitch;
  float yaw;
  float altitude;
} control_commands_t;

typedef struct desired_state_s {
  float roll;
  float pitch;
  float yaw_rate;
  float altitude;
  float vx;
  float vy;
} desired_state_t;

typedef struct actual_state_s {
  float roll;
  float pitch;
  float yaw_rate;
  float altitude;
  float vx;
  float vy;
} actual_state_t;

typedef struct gains_pid_s {
  float kp_att_rp;
  float kd_att_rp;
  float kp_att_y;
  float kd_att_y;
  float kp_vel_xy;
  float kd_vel_xy;
  float kp_z;
  float kd_z;
  float ki_z;
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
                                          float dt, motor_power_t *motorCommands);
void pid_velocity_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                          float dt, motor_power_t *motorCommands);
void pid_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, float dt,
                                 control_commands_t *control_commands);
void pid_attitude_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, float dt,
                             control_commands_t *control_commands);
void pid_horizontal_velocity_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                        float dt);
void motor_mixing(control_commands_t control_commands, motor_power_t *motorCommands);
void calculate_thrust_torques(const motor_power_t *motorCommands, float *T, float *Tx, float *Ty, float *Tz);
void resetControlIfThrustZero(control_t *control);

#endif //__CONTROLLER_CustomPID_H__