#ifndef __CONTROLLER_GenericCustom_H__
#define __CONTROLLER_GenericCustom_H__
#include "stabilizer_types.h"

// -------------------------- //
// MAPPING SIMULATION TO REAL 
// -------------------------- //


typedef struct control_commands_s {
  float roll;
  float pitch;
  float yaw;
  float altitude;
} control_commands_t;

typedef struct desired_state_s {
  // shared
  float roll;
  float pitch;
  float yaw;

  // For direct thrust PID calculations
  float x;
  float y;
  float z;

  float vx;
  float vy;
  float vz;
  float vroll;
  float vpitch;
  float vyaw;
  


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

typedef struct current_state_s {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float vx;
    float vy;
    float vz;
    float vroll;
    float vpitch; 
    float vyaw;
} current_state_t;

// -------------------------- //
// Custom Direct Thrust controller types
// -------------------------- //

typedef struct {
  // PID gains for roll, pitch, yaw
  float kp_roll, ki_roll, kd_roll;
  float kp_pitch, ki_pitch, kd_pitch;
  float kp_yaw, ki_yaw, kd_yaw;
  float kp_x, ki_x, kd_x;
  float kp_y, ki_y, kd_y;

  // PID gains for thrust
  float kp_thrust, ki_thrust, kd_thrust;

  // Integral term limits
  float integral_max_roll, integral_max_pitch, integral_max_yaw, integral_max_thrust;
  float integral_max_x, integral_max_y;

  // Output limits
  float output_max_roll, output_max_pitch, output_max_yaw, output_max_thrust;
  float output_max_x, output_max_y;
  float output_min_roll, output_min_pitch, output_min_yaw, output_min_thrust;
  float output_min_x, output_min_y;

} CustomDirectThrust_t;

typedef struct {
    float kp, ki, kd;
    float integral_max, output_max, output_min;
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

typedef struct {
    float x;
    float y;
    float z;
} Vector3f;
// -------------------------- //
// FUTURE TYPES EXPLAIN 
// -------------------------- //

// SMC params controller 5 params
typedef struct {
    float nu;
    float g;        // Gravity constant
    float A;        // Drag constant
    float m;        // Mass of the quadrotor
    float lambda_z;
    float lambda_roll;
    float lambda_pitch;
    float lambda_yaw;

    float kD_z;
    float kD_roll;
    float kD_pitch;
    float kD_yaw;

    float delta_z;
    float delta_roll;
    float delta_pitch;
    float delta_yaw;

    float I_xx;
    float I_yy;
    float I_zz;
    float Jr;
} SMC5_params_t;

typedef struct {
    float nu;
    float g;
    float A; // Drag constant
    float m; // Mass of the quadrotor
    float alpha_z; // Sliding mode control parameter original 4.0
    float alpha_roll;
    float alpha_pitch;
    float I_xx;
    float I_yy;
    float I_zz;
    float gain_z;
    float gain_roll;
    float gain_pitch;
    float gain_yaw;

    float roll_k1;
    float roll_k2;
    float roll_eps;

    float pitch_k1;
    float pitch_k2;
    float pitch_eps;
    
    
} SMC1_params_t;


#endif
