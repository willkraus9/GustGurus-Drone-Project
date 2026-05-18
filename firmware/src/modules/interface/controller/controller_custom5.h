// #ifndef __CONTROLLER_DirectThrustPID_H__
// #define __CONTROLLER_DirectThrustPID_H__
// #include "stabilizer_types.h"
// #include "controller_custom_types.h"

// #ifdef CRAZYFLIE_FW
// /** Firmware initialization */
// void controllerCustomFirmware5Init(void);

// /** Firmware test */
// bool controllerCustomFirmware5Test(void);

// /** Firmware control */
// void controllerCustomFirmware5(
//     control_t *control,
//     const setpoint_t *setpoint,
//     const sensorData_t *sensors,
//     const state_t *state,
//     const uint32_t stabilizerStep
// );

// float smc5_altitude(float error_z, float error_d_z, float desired_acc_z, float roll, float pitch, SMC5_params_t params);
// float smc5_roll(float error_roll, float error_d_roll, float desired_acc_roll, float roll_rate, float yaw_rate, float omega, SMC5_params_t params);
// float smc5_pitch(float error_pitch, float error_d_pitch, float desired_acc_pitch, float roll_rate, float yaw_rate, float omega, SMC5_params_t params);
// float smc5_yaw(float error_yaw, float error_d_yaw, float desired_acc_yaw, float roll_rate, float pitch_rate, SMC5_params_t params);

// void map_setpoint_to_desired_direct_thrust_smc5(const setpoint_t *setpoint, desired_state_t *desired_state);
// void map_state_to_actual_direct_thrust_smc5(const state_t *state, const sensorData_t *sensors, current_state_t *current_state);
// float smc5_pitch(float error_pitch, float error_d_pitch, float desired_acc_pitch, float roll_rate, float yaw_rate, float omega, SMC5_params_t params);
// float get_omega(float thrust, float thrustX, float thrustY, float thrustZ);

// #endif // CRAZYFLIE_FW

// #endif // CONTROLLER_CUSTOMPID_DIRECT_THRUST_H__
