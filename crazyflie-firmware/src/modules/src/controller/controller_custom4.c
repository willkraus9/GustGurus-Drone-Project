/*
// __        _____ _   _ ____  ____  ____  _____    _    _  _______ ____  ____  
// \ \      / /_ _| \ | |  _ \| __ )|  _ \| ____|  / \  | |/ / ____|  _ \/ ___| 
//  \ \ /\ / / | ||  \| | | | |  _ \| |_) |  _|   / _ \ | ' /|  _| | |_) \___ \ 
//   \ V  V /  | || |\  | |_| | |_) |  _ <| |___ / ___ \| . \| |___|  _ < ___) |
//    \_/\_/  |___|_| \_|____/|____/|_| \_\_____/_/   \_\_|\_\_____|_| \_\____/ 
//  
//  Crazyflie control firmware
//  
//  Custom implementation of a sliding mode controller by Nikolaj Hindsbo & Denis Alpay
//  
*/

#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>

#include "stabilizer_types.h"
#include "attitude_controller.h"
#include "position_controller.h"
#include "param.h"
#include "math3d.h"
#include <string.h>
#include <stdint.h>
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"

#include "controller.h"
#include "controller_custom4.h"
#include <stdbool.h>
#define constrain(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

#ifdef CRAZYFLIE_FW
// Firmware initialization
void controllerCustomFirmware4Init(void) {
    // todo link to the init for LQR. 
}

// Firmware test
bool controllerCustomFirmware4Test(void) {
    return true;
}

// Firmware control
void controllerCustomFirmware4(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
) {
    // todo link to the main update function for LQR.
}
#endif

#ifdef CRAZYFLIE_FW
#include "param.h"
#include "log.h"

PARAM_GROUP_START(ctrlCustom4)

PARAM_GROUP_STOP(ctrlCustom4)

LOG_GROUP_START(ctrlCustom4)

LOG_GROUP_STOP(ctrlCustom4)

#endif // CRAZYFLIE_FW defined