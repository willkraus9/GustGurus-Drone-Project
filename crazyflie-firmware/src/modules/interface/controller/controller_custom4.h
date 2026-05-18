// #ifndef __CONTROLLER_DirectThrustPID_H__
// #define __CONTROLLER_DirectThrustPID_H__
// #include "stabilizer_types.h"
// #include "controller_custom_types.h"

// #ifdef CRAZYFLIE_FW
// /** Firmware initialization */
// void controllerCustomFirmware4Init(void);

// /** Firmware test */
// bool controllerCustomFirmware4Test(void);

// /** Firmware control */
// void controllerCustomFirmware4(
//     control_t *control,
//     const setpoint_t *setpoint,
//     const sensorData_t *sensors,
//     const state_t *state,
//     const uint32_t stabilizerStep
// );

// // static void matMult(const double* A, const double* B, double* C, int n, int k, int m);
// // static void matAdd(const double* A, const double* B, double* C, int n, int m);
// // static void matSub(const double* A, const double* B, double* C, int n, int m);
// // static void matTrans(const double* A, double* C, int n, int m);
// // static void matCopy(const double* A, double* B, int n, int m);
// // static double matNorm(const double* A, int n, int m);
// // static bool svd(const double* A, int m, int n, double* U, double* S, double* V);
// // static void pseudoInverse(const double* A, double* A_pinv, int m, int n);
// #endif // CRAZYFLIE_FW

// #endif // CONTROLLER_CUSTOMPID_DIRECT_THRUST_H__