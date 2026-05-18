// /*
// // __        _____ _   _ ____  ____  ____  _____    _    _  _______ ____  ____  
// // \ \      / /_ _| \ | |  _ \| __ )|  _ \| ____|  / \  | |/ / ____|  _ \/ ___| 
// //  \ \ /\ / / | ||  \| | | | |  _ \| |_) |  _|   / _ \ | ' /|  _| | |_) \___ \
// //   \ V  V /  | || |\  | |_| | |_) |  _ <| |___ / ___ \| . \| |___|  _ < ___) |
// //    \_/\_/  |___|_| \_|____/|____/|_| \_\_____/_/   \_\_|\_\_____|_| \_\____/ 
// //
// //  Crazyflie control firmware
// //  
// //  Custom implementation of a LQR Controller by Nikolaj Hindsbo & Denis Alpay
// */

// #include "controller_custom3.h"
// #include "param.h"
// #include "log.h"
// #include "math3d.h"
// #include "num.h"
// #include "eeprom.h"
// #include "console.h"
// #include <string.h>
// #include <math.h>
// #include "FreeRTOS.h"
// #include "debug.h"
// #include "controller_custom_types.h"
// #include <stdbool.h>
// #include <math.h>
// #include <string.h>
// #include <stdbool.h>
// #include <stdio.h>


// #define __DEBUG_COORDS 0

// // Constants
// static const float MASS = 0.0313f; // kg
// static const float GRAVITY = 9.81f; // m/s^2
// #define TOL 1e-6
// #define MAX_ITER 100

// // System matrices
// // static float LQR_A[12][12] = {
// //         {1, 0, 0, 0, 1.962e-5, 0, 0.002, 0, 0, 0, 1.308e-8, 0},
// //         {0, 1, 0, -1.962e-5, 0, 0, 0, 0.002, 0, -1.308e-8, 0, 0},
// //         {0, 0, 1, 0, 0, 0, 0, 0, 0.002, 0, 0, 0},
// //         {0, 0, 0, 1, 0, 0, 0, 0, 0, 0.002, 0, 0},
// //         {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0.002, 0},
// //         {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0.002},
// //         {0, 0, 0, 0, 0.01962, 0, 1, 0, 0, 0, 1.962e-5, 0},
// //         {0, 0, 0, -0.01962, 0, 0, 0, 1, 0, -1.962e-5, 0, 0},
// //         {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
// //         {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
// //         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
// //         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}
// // };

// // static float LQR_B[12][4] = {
// //         {0, 0, 2.73057492380277e-7, 0},
// //         {0, -2.73057492380277e-7, 0, 0},
// //         {6.38977635782748e-5, 0, 0, 0},
// //         {0, 0.0835038202997787, 0, 0},
// //         {0, 0, 0.0835038202997787, 0},
// //         {0, 0, 0, 0.0618295359693326},
// //         {0, 0, 0.000546114984760553, 0},
// //         {0, -0.000546114984760553, 0, 0},
// //         {0.0638977635782748, 0, 0, 0},
// //         {0, 83.5038202997787, 0, 0},
// //         {0, 0, 83.5038202997787, 0},
// //         {0, 0, 0, 61.8295359693326}
// // };

// // LQR gain matrix
// static float LQR_K[4][12] = {0};

// // LQR parameters
// static float LQR_Q[12][12] = {
//     {1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0}
// };

// static float LQR_R[4][4] = {
//     {1.0, 0, 0, 0},
//     {0, 1.0, 0, 0},
//     {0, 0, 1.0, 0},
//     {0, 0, 0, 1.0}
// };

// // State variables
// static float state_vec[12];
// static float setpt_vec[12];
// static float state_error_vec[12];
// static float input_vec[4];

// /*** Initialize the LQR controller ***/
// void controllerCustomFirmware3Init() {
//     DEBUG_PRINT("NEW Custom LQR Controller Initialized\n");
//     solveRiccatiEquation();
//     // Debug print the LQR gain matrix
//     DEBUG_PRINT("Initialized k matrix ");
//     // Debug print the LQR gain matrix
//     DEBUG_PRINT("LQR Gain Matrix (K):\n");
//     for (int i = 0; i < 4; i++) { // Iterate over rows
//         DEBUG_PRINT("Row %d: ", i);
//         for (int j = 0; j < 12; j++) { // Iterate over columns
//             DEBUG_PRINT("%.15f ", (double)LQR_K[i][j]);
//         }
//         DEBUG_PRINT("\n"); // Newline after each row
//     }
// }

// // Firmware test
// bool controllerCustomFirmware3Test() {
//     return true;
// }

// /*** Dynamically calculate K matrix ***/
// void calculateKMatrix() {
//     solveRiccatiEquation();
// }

// /*** Solve Riccati Equation and calculate K ***/
// void solveRiccatiEquation() {
//     // double P[12 * 12]; // This should come from a real Riccati solver
//     // double BR[4 * 4]; // Inverse of R
//     // double BT_P[4 * 12]; // B^T * P
//     // double BT[4 * 12];

//     // double A_flat[12 * 12];
//     // double B_flat[12 * 4];
//     // double Q_flat[12 * 12];
//     // double R_flat[4 * 4];

//     // // Copy LQR_A into A_flat as double
//     // for (int row = 0; row < 12; row++) {
//     //     for (int col = 0; col < 12; col++) {
//     //         A_flat[row * 12 + col] = (double)LQR_A[row][col];
//     //     }
//     // }

//     // // Copy LQR_B into B_flat as double
//     // for (int row = 0; row < 12; row++) {
//     //     for (int col = 0; col < 4; col++) {
//     //         B_flat[row * 4 + col] = (double)LQR_B[row][col];
//     //     }
//     // }

//     // // Copy LQR_Q into Q_flat
//     // for (int row = 0; row < 12; row++) {
//     //     for (int col = 0; col < 12; col++) {
//     //         Q_flat[row * 12 + col] = (double)LQR_Q[row][col];
//     //     }
//     // }

//     // // Copy LQR_R into R_flat
//     // for (int row = 0; row < 4; row++) {
//     //     for (int col = 0; col < 4; col++) {
//     //         R_flat[row * 4 + col] = (double)LQR_R[row][col];
//     //     }
//     // }

//     // // Invert R
//     // if (!invertMatrixCholesky(R_flat, 4, BR)) {
//     //     DEBUG_PRINT("Error: Unable to invert R using Cholesky.\n");
//     //     return;
//     // }

//     // // For now, just set P = Q to have something (not correct LQR!)
//     // for (int i = 0; i < 12 * 12; ++i) {
//     //     P[i] = Q_flat[i]; 
//     // }

//     // // Compute B^T
//     // for (int i = 0; i < 12; i++) {
//     //     for (int j = 0; j < 4; j++) {
//     //         BT[j * 12 + i] = B_flat[i * 4 + j];
//     //     }
//     // }

//     // // Compute B^T * P
//     // for (int i = 0; i < 4; i++) {
//     //     for (int j = 0; j < 12; j++) {
//     //         double sum = 0.0;
//     //         for (int k = 0; k < 12; k++) {
//     //             sum += BT[i * 12 + k] * P[k * 12 + j];
//     //         }
//     //         BT_P[i * 12 + j] = sum;
//     //     }
//     // }

//     // Compute K = R^-1 * B^T * P
//     for (int i = 0; i < 4; i++) {
//         for (int j = 0; j < 12; j++) {
//             // double sum = 0.0;
//             for (int k = 0; k < 4; k++) {
//                 // sum += BR[i * 4 + k] * BT_P[k * 12 + j];
//             }
//             LQR_K[i][j] = 1;
//         }
//     }

//     DEBUG_PRINT("Computed K matrix\n");
// }





// /*** LQR control logic ***/
// void controllerCustomFirmware3(control_t *control, const setpoint_t *setpoint,
//                                const sensorData_t *sensors, const state_t *state,
//                                const uint32_t stabilizerStep) {
//    // Keep this function the same 
//     if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
//         return;
//     }

//     static uint32_t lastPrintTime = 0;
//     uint32_t currentTime = xTaskGetTickCount();
//     control->controlMode = controlModeForceTorque;

//     desired_state_t desired_state;
//     current_state_t current_state;

//     map_setpoint_to_desired_direct_thrust_LQR(setpoint, &desired_state);
//     map_state_to_actual_direct_thrust_LQR(state, sensors, &current_state);

//     bool isToppled = current_state.roll > 1.0f || current_state.roll < -1.0f || current_state.pitch > 1.0f || current_state.pitch < -1.0f;
//     // CHANGED  
//     state_vec[0]  = state->position.x;
//     state_vec[1]  = state->position.y;
//     state_vec[2]  = state->position.z;
//     state_vec[3]  = radians(state->attitude.roll);
//     state_vec[4]  = -radians(state->attitude.pitch);
//     state_vec[5]  = radians(state->attitude.yaw);

//     // CHANGED  
//     state_vec[6]  = state->velocity.x;
//     state_vec[7]  = state->velocity.y;
//     state_vec[8]  = state->velocity.z;
//     state_vec[9]  = radians(sensors->gyro.x);
//     state_vec[10] = -radians(sensors->gyro.y);
//     state_vec[11] = radians(sensors->gyro.z);

//     setpt_vec[0]  = setpoint->position.x;
//     setpt_vec[1]  = setpoint->position.y;
//     setpt_vec[2]  = setpoint->position.z;
//     setpt_vec[3]  = radians(setpoint->attitude.roll);
//     setpt_vec[4]  = -radians(setpoint->attitude.pitch);
//     setpt_vec[5]  = radians(setpoint->attitude.yaw);
//     setpt_vec[6]  = setpoint->velocity.x;
//     setpt_vec[7]  = setpoint->velocity.y;
//     setpt_vec[8]  = setpoint->velocity.z;
//     setpt_vec[9]  = radians(setpoint->attitudeRate.roll);
//     setpt_vec[10] = -radians(setpoint->attitudeRate.pitch);
//     setpt_vec[11] = radians(setpoint->attitudeRate.yaw);

//     for (int idx = 0; idx < 12; idx++){
//          state_error_vec[idx] = state_vec[idx] - setpt_vec[idx];
//     }

//     // Matrix multiplication!
//     for (int i = 0; i < 4; i++) {
//         input_vec[i] = 0;
//         for (int j = 0; j < 12; j++) {
//             input_vec[i] -= LQR_K[i][j] * state_error_vec[j];
//         }
//     }

//     input_vec[0] += setpoint->thrust + MASS * GRAVITY;

//     control->thrustSi = input_vec[0];
//     control->torqueX = input_vec[1];
//     control->torqueY = input_vec[2];
//     control->torqueZ = input_vec[3];

//     if (setpt_vec[2] < 0.1f && state_vec[2] < 0.3f) {
//         control->thrustSi = 0.0f;
//         control->torqueX = 0.0f;
//         control->torqueY = 0.0f;
//         control->torqueZ = 0.0f;
//     }
    
//     if (currentTime - lastPrintTime >= pdMS_TO_TICKS(1000) && !isToppled && desired_state.z > 0.1f) {
//         DEBUG_PRINT("----------------------------------------------------------------------\n");
//         DEBUG_PRINT("Thrust: %.3f, TorqueX: %.6f, TorqueY: %.6f, TorqueZ: %.6f\n", (double)input_vec[0], (double)input_vec[1], (double)input_vec[2], (double)input_vec[3]);
//         lastPrintTime = currentTime;
//     }
// }

// void map_setpoint_to_desired_direct_thrust_LQR(const setpoint_t *setpoint, desired_state_t *desired_state) {
//     desired_state->x = setpoint->position.x;
//     desired_state->y = setpoint->position.y;
//     desired_state->z = setpoint->position.z;
//     desired_state->roll = radians(setpoint->attitude.roll);
//     desired_state->pitch = -radians(setpoint->attitude.pitch);
//     desired_state->yaw = radians(setpoint->attitude.yaw);
//     desired_state->vx = setpoint->velocity.x;
//     desired_state->vy = setpoint->velocity.y;
//     desired_state->vz = setpoint->velocity.z;
//     desired_state->vroll = radians(setpoint->attitudeRate.roll);
//     desired_state->vpitch = -radians(setpoint->attitudeRate.pitch);
//     desired_state->vyaw = radians(setpoint->attitudeRate.yaw);
// }

// // Map the current state
// void map_state_to_actual_direct_thrust_LQR(const state_t *state, const sensorData_t *sensors, current_state_t *current_state) {
//     current_state->roll = state->attitude.roll / 57.3f;
//     current_state->pitch = state->attitude.pitch / 57.3f;
//     current_state->yaw = state->attitude.yaw / 57.3f;
//     current_state->z = state->position.z;
//     current_state->x = state->position.x;
//     current_state->y = state->position.y;
//     current_state->vx = state->velocity.x;
//     current_state->vy = state->velocity.y;
//     current_state->vz = state->velocity.z;
//     current_state->vroll = sensors->gyro.x;
//     current_state->vpitch = sensors->gyro.y;
//     current_state->vyaw = sensors->gyro.z;
// }

// /*** Custom Cholesky-based Matrix Inversion ***/
// bool invertMatrixCholesky(const double *matrix, int size, double *inverse) {
//     double L[size * size];
//     memset(L, 0, sizeof(double) * size * size);

//     // Compute Cholesky decomposition
//     for (int i = 0; i < size; i++) {
//         for (int j = 0; j <= i; j++) {
//             double sum = 0.0;
//             for (int k = 0; k < j; k++) {
//                 sum += L[i * size + k] * L[j * size + k];
//             }
//             if (i == j) {
//                 L[i * size + j] = sqrt(matrix[i * size + i] - sum);
//             } else {
//                 L[i * size + j] = (matrix[i * size + j] - sum) / L[j * size + j];
//             }
//         }
//     }

//     // Compute L^-1
//     double L_inv[size * size];
//     memset(L_inv, 0, sizeof(double) * size * size);
//     for (int i = 0; i < size; i++) {
//         L_inv[i * size + i] = 1.0 / L[i * size + i];
//         for (int j = i + 1; j < size; j++) {
//             double sum = 0.0;
//             for (int k = i; k < j; k++) {
//                 sum -= L[j * size + k] * L_inv[k * size + i];
//             }
//             L_inv[j * size + i] = sum / L[j * size + j];
//         }
//     }

//     // Compute inverse = L^-T * L^-1
//     memset(inverse, 0, sizeof(double) * size * size);
//     for (int i = 0; i < size; i++) {
//         for (int j = 0; j < size; j++) {
//             for (int k = 0; k <= j; k++) {
//                 inverse[i * size + j] += L_inv[k * size + i] * L_inv[k * size + j];
//             }
//         }
//     }
//     return true;
// }


// PARAM_GROUP_START(ctrlLQR)
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, Q_x, &LQR_Q[0][0])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, Q_y, &LQR_Q[1][1])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, Q_z, &LQR_Q[2][2])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, Q_roll, &LQR_Q[3][3])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, Q_pitch, &LQR_Q[4][4])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, Q_yaw, &LQR_Q[5][5])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, dQ_x, &LQR_Q[6][6])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, dQ_y, &LQR_Q[7][7])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, dQ_z, &LQR_Q[8][8])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, dQ_roll, &LQR_Q[9][9])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, dQ_pitch, &LQR_Q[10][10])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, dQ_yaw, &LQR_Q[11][11])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, R_thrust, &LQR_R[0][0])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, R_torqueX, &LQR_R[1][1])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, R_torqueY, &LQR_R[2][2])
// PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, R_torqueZ, &LQR_R[3][3])
// PARAM_GROUP_STOP(ctrlLQR)

// // /*** Logging parameters ***/
// // LOG_GROUP_START(ctrlLQR)
// // LOG_ADD(LOG_FLOAT, Q_x, &Q[0][0])
// // LOG_ADD(LOG_FLOAT, Q_y, &Q[1][1])
// // LOG_ADD(LOG_FLOAT, Q_z, &Q[2][2])
// // LOG_ADD(LOG_FLOAT, Q_roll, &Q[3][3])
// // LOG_ADD(LOG_FLOAT, Q_pitch, &Q[4][4])
// // LOG_ADD(LOG_FLOAT, Q_yaw, &Q[5][5])
// // LOG_ADD(LOG_FLOAT, dQ_x, &dQ[0][0])
// // LOG_ADD(LOG_FLOAT, dQ_y, &dQ[1][1])
// // LOG_ADD(LOG_FLOAT, dQ_z, &dQ[2][2])
// // LOG_ADD(LOG_FLOAT, dQ_roll, &dQ[3][3])
// // LOG_ADD(LOG_FLOAT, dQ_pitch, &dQ[4][4])
// // LOG_ADD(LOG_FLOAT, dQ_yaw, &dQ[5][5])
// // LOG_ADD(LOG_FLOAT, R_thrust, &R[0][0])
// // LOG_ADD(LOG_FLOAT, R_torqueX, &R[1][1])
// // LOG_ADD(LOG_FLOAT, R_torqueY, &R[2][2])
// // LOG_ADD(LOG_FLOAT, R_torqueZ, &R[3][3])
// // LOG_GROUP_STOP(ctrlLQR)

