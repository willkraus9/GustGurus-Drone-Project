#ifndef __CONTROLLER_DirectThrustPID_H__
#define __CONTROLLER_DirectThrustPID_H__
#include "stabilizer_types.h"
#include "controller_custom_types.h"

#ifdef CRAZYFLIE_FW
/** Firmware initialization */
void controllerCustomFirmware4Init(void);

/** Firmware test */
bool controllerCustomFirmware4Test(void);

/** Firmware control */
void controllerCustomFirmware4(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
);
#endif // CRAZYFLIE_FW

#endif // CONTROLLER_CUSTOMPID_DIRECT_THRUST_H__