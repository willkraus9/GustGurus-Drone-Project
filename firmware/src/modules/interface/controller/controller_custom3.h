#ifndef _CONTROLLER_DirectThrustPID_H_
#define _CONTROLLER_DirectThrustPID_H_

#include "stabilizer_types.h"
#include "controller_custom_types.h"

#ifdef CRAZYFLIE_FW
/* Firmware initialization */
void controllerCustomFirmware3Init(void);

/* Firmware test */
bool controllerCustomFirmware3Test(void);

/* Firmware control */
void controllerCustomFirmware3(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep);

// Map setpoint to desired state
void map_setpoint_to_desired_direct_thrust_LQR(const setpoint_t *setpoint, desired_state_t *desired_state);

// Map state to actual state
void map_state_to_actual_direct_thrust_LQR(const state_t *state, const sensorData_t *sensors, current_state_t *current_state);
#endif // CRAZYFLIE_FW

#endif // _CONTROLLER_DirectThrustPID_H_

// changes from new LQR controller: 

// // static float capAngle(float angle);
// void controllerLqrInit(void);
// void controllerLqrInit(void);
// bool controllerLqrTest(void);
// void controllerLqr(control_t *control, const setpoint_t *setpoint,
//                    const sensorData_t *sensors, const state_t *state,
//                    const uint32_t tick);






