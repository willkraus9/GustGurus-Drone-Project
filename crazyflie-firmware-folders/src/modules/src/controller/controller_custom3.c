/*
// __        _____ _   _ ____  ____  ____  _____    _    _  _______ ____  ____  
// \ \      / /_ _| \ | |  _ \| __ )|  _ \| ____|  / \  | |/ / ____|  _ \/ ___| 
//  \ \ /\ / / | ||  \| | | | |  _ \| |_) |  _|   / _ \ | ' /|  _| | |_) \___ \
//   \ V  V /  | || |\  | |_| | |_) |  _ <| |___ / ___ \| . \| |___|  _ < ___) |
//    \_/\_/  |___|_| \_|____/|____/|_| \_\_____/_/   \_\_|\_\_____|_| \_\____/ 
//
//  Crazyflie control firmware
//  
//  Custom implementation of a LQR Controller by Nikolaj Hindsbo & Denis Alpay
*/
#include "controller_custom3.h"
#include "param.h"
#include "log.h"
#include "math3d.h"
#include "num.h"
#include "eeprom.h"
#include "console.h"
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "debug.h"
#include "controller_custom_types.h"

#define __DEBUG_COORDS 0

static const float MASS    = 0.0313f;     // kg
static const float GRAVITY = 9.81f;     // m / s^2


static const float L       = 0.046f;    // m

static float K[4][12] = {
    {-1.89814894e-13,  0.00000000e+00,  1.22943546e+01,  0.00000000e+00,
     -2.99770293e-12,  0.00000000e+00, -3.61073430e-13,  0.00000000e+00,
      8.77821904e-01,  0.00000000e+00, -2.33725196e-12,  0.00000000e+00},
    { 0.00000000e+00, -1.59383354e-03,  0.00000000e+00,  1.48803678e-02,
      0.00000000e+00,  1.36528461e-11,  0.00000000e+00, -2.19927784e-03,
      0.00000000e+00,  5.14554485e-03,  0.00000000e+00,  7.18928267e-14},
    { 1.59383355e-03,  0.00000000e+00, -2.28114205e-13,  0.00000000e+00,
      1.48803679e-02,  0.00000000e+00,  2.19927786e-03,  0.00000000e+00,
     -8.70169129e-15,  0.00000000e+00,  5.14554485e-03,  0.00000000e+00},
    { 0.00000000e+00, -1.31704745e-12,  0.00000000e+00,  1.60295462e-11,
      0.00000000e+00,  4.06755758e-02,  0.00000000e+00, -2.04984516e-12,
      0.00000000e+00,  5.91393329e-14,  0.00000000e+00,  5.66081644e-03}
};

static float K_F     = 1.8221205267714052e-06f;  //1.938952e-6f; // N / "PWM"
static float K_M     = 4.4732910188601685e-08f; // Nm / "PWM"
static float state_vec[12];
static float setpt_vec[12];
static float state_error_vec[12];
static float input_vec[4];
static float pwm_vec[4];
static int16_t pwm_total;
static int16_t pwm_int[3];
static float force_total;
static float thrust;
static float set_pitch;
static uint8_t idx = 0;



/***
 * Initialize the LQR controller.
 * Called once when switching to the LQR controller. Useful for initializing
 * static variables used by the module.
 */
void controllerCustomFirmware3Init(void) {
    DEBUG_PRINT("NEW Custom LQR Controller Initialized\n");
}

/***
 * Test the LQR controller initialization.
 * Called by the system to check that the controller is properly initialized.
 */
bool controllerCustomFirmware3Test(void) {
    return true;
}

void controllerCustomFirmware3(control_t *control, const setpoint_t *setpoint,
                               const sensorData_t *sensors,
                               const state_t *state,
                               const uint32_t stabilizerStep) {
    if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
        return;
    }

    static uint32_t lastPrintTime = 0;
    static uint32_t lastPrintTime2 = 0;
    uint32_t currentTime = xTaskGetTickCount();
    control->controlMode = controlModeForceTorque;

    desired_state_t desired_state;
    current_state_t current_state;

    map_setpoint_to_desired_direct_thrust_LQR(setpoint, &desired_state);
    map_state_to_actual_direct_thrust_LQR(state, sensors, &current_state);

    bool isToppled = current_state.roll > 1.0f || current_state.roll < -1.0f || current_state.pitch > 1.0f || current_state.pitch < -1.0f;
    // CHANGED  
    state_vec[0]  = state->position.x;
    state_vec[1]  = state->position.y;
    state_vec[2]  = state->position.z;
    state_vec[3]  = radians(state->attitude.roll);
    state_vec[4]  = -radians(state->attitude.pitch);
    state_vec[5]  = radians(state->attitude.yaw);

    // CHANGED  
    state_vec[6]  = state->velocity.x;
    state_vec[7]  = state->velocity.y;
    state_vec[8]  = state->velocity.z;
    state_vec[9]  = radians(sensors->gyro.x);
    state_vec[10] = -radians(sensors->gyro.y);
    state_vec[11] = radians(sensors->gyro.z);

    setpt_vec[0]  = setpoint->position.x;
    setpt_vec[1]  = setpoint->position.y;
    setpt_vec[2]  = setpoint->position.z;
    setpt_vec[3]  = radians(setpoint->attitude.roll);
    setpt_vec[4]  = -radians(setpoint->attitude.pitch);
    setpt_vec[5]  = radians(setpoint->attitude.yaw);
    setpt_vec[6]  = setpoint->velocity.x;
    setpt_vec[7]  = setpoint->velocity.y;
    setpt_vec[8]  = setpoint->velocity.z;
    setpt_vec[9]  = radians(setpoint->attitudeRate.roll);
    setpt_vec[10] = -radians(setpoint->attitudeRate.pitch);
    setpt_vec[11] = radians(setpoint->attitudeRate.yaw);

    for (int idx = 0; idx < 12; idx++){
         state_error_vec[idx] = state_vec[idx] - setpt_vec[idx];
    }

    // Matrix multiplication!
    for (int i = 0; i < 4; i++) {
        input_vec[i] = 0;
        for (int j = 0; j < 12; j++) {
            input_vec[i] -= K[i][j] * state_error_vec[j];
        }
    }

    input_vec[0] += setpoint->thrust + MASS * GRAVITY;

    control->thrustSi = input_vec[0];
    control->torqueX = input_vec[1];
    control->torqueY = input_vec[2];
    control->torqueZ = input_vec[3];

    if (setpt_vec[2] < 0.1f && state_vec[2] < 0.3f) {
        control->thrustSi = 0.0f;
        control->torqueX = 0.0f;
        control->torqueY = 0.0f;
        control->torqueZ = 0.0f;
    }
    
    if (currentTime - lastPrintTime >= pdMS_TO_TICKS(1000) && !isToppled && desired_state.z > 0.1f) {
        DEBUG_PRINT("----------------------------------------------------------------------\n");
        DEBUG_PRINT("Thrust: %.3f, TorqueX: %.6f, TorqueY: %.6f, TorqueZ: %.6f\n", (double)input_vec[0], (double)input_vec[1], (double)input_vec[2], (double)input_vec[3]);
        lastPrintTime = currentTime;
    }
}

void map_setpoint_to_desired_direct_thrust_LQR(const setpoint_t *setpoint, desired_state_t *desired_state) {
    desired_state->x = setpoint->position.x;
    desired_state->y = setpoint->position.y;
    desired_state->z = setpoint->position.z;
    desired_state->roll = radians(setpoint->attitude.roll);
    desired_state->pitch = -radians(setpoint->attitude.pitch);
    desired_state->yaw = radians(setpoint->attitude.yaw);
    desired_state->vx = setpoint->velocity.x;
    desired_state->vy = setpoint->velocity.y;
    desired_state->vz = setpoint->velocity.z;
    desired_state->vroll = radians(setpoint->attitudeRate.roll);
    desired_state->vpitch = -radians(setpoint->attitudeRate.pitch);
    desired_state->vyaw = radians(setpoint->attitudeRate.yaw);
}

// Map the current state
void map_state_to_actual_direct_thrust_LQR(const state_t *state, const sensorData_t *sensors, current_state_t *current_state) {
    current_state->roll = state->attitude.roll / 57.3f;
    current_state->pitch = state->attitude.pitch / 57.3f;
    current_state->yaw = state->attitude.yaw / 57.3f;
    current_state->z = state->position.z;
    current_state->x = state->position.x;
    current_state->y = state->position.y;
    current_state->vx = state->velocity.x;
    current_state->vy = state->velocity.y;
    current_state->vz = state->velocity.z;
    current_state->vroll = sensors->gyro.x;
    current_state->vpitch = sensors->gyro.y;
    current_state->vyaw = sensors->gyro.z;
}

PARAM_GROUP_START(ctrlLQR)
PARAM_ADD(PARAM_FLOAT, k_f, &K_F)
PARAM_ADD(PARAM_FLOAT, k_m, &K_M)
PARAM_ADD(PARAM_FLOAT, k11, &K[0][0])
PARAM_ADD(PARAM_FLOAT, k21, &K[1][0])
PARAM_ADD(PARAM_FLOAT, k31, &K[2][0])
PARAM_ADD(PARAM_FLOAT, k41, &K[3][0])
PARAM_ADD(PARAM_FLOAT, k12, &K[0][1])
PARAM_ADD(PARAM_FLOAT, k22, &K[1][1])
PARAM_ADD(PARAM_FLOAT, k32, &K[2][1])
PARAM_ADD(PARAM_FLOAT, k42, &K[3][1])
PARAM_ADD(PARAM_FLOAT, k13, &K[0][2])
PARAM_ADD(PARAM_FLOAT, k23, &K[1][2])
PARAM_ADD(PARAM_FLOAT, k33, &K[2][2])
PARAM_ADD(PARAM_FLOAT, k43, &K[3][2])
PARAM_ADD(PARAM_FLOAT, k14, &K[0][3])
PARAM_ADD(PARAM_FLOAT, k24, &K[1][3])
PARAM_ADD(PARAM_FLOAT, k34, &K[2][3])
PARAM_ADD(PARAM_FLOAT, k44, &K[3][3])
PARAM_ADD(PARAM_FLOAT, k15, &K[0][4])
PARAM_ADD(PARAM_FLOAT, k25, &K[1][4])
PARAM_ADD(PARAM_FLOAT, k35, &K[2][4])
PARAM_ADD(PARAM_FLOAT, k45, &K[3][4])
PARAM_ADD(PARAM_FLOAT, k16, &K[0][5])
PARAM_ADD(PARAM_FLOAT, k26, &K[1][5])
PARAM_ADD(PARAM_FLOAT, k36, &K[2][5])
PARAM_ADD(PARAM_FLOAT, k46, &K[3][5])
PARAM_ADD(PARAM_FLOAT, k17, &K[0][6])
PARAM_ADD(PARAM_FLOAT, k27, &K[1][6])
PARAM_ADD(PARAM_FLOAT, k37, &K[2][6])
PARAM_ADD(PARAM_FLOAT, k47, &K[3][6])
PARAM_ADD(PARAM_FLOAT, k18, &K[0][7])
PARAM_ADD(PARAM_FLOAT, k28, &K[1][7])
PARAM_ADD(PARAM_FLOAT, k38, &K[2][7])
PARAM_ADD(PARAM_FLOAT, k48, &K[3][7])
PARAM_ADD(PARAM_FLOAT, k19, &K[0][8])
PARAM_ADD(PARAM_FLOAT, k29, &K[1][8])
PARAM_ADD(PARAM_FLOAT, k39, &K[2][8])
PARAM_ADD(PARAM_FLOAT, k49, &K[3][8])
PARAM_ADD(PARAM_FLOAT, k110, &K[0][9])
PARAM_ADD(PARAM_FLOAT, k210, &K[1][9])
PARAM_ADD(PARAM_FLOAT, k310, &K[2][9])
PARAM_ADD(PARAM_FLOAT, k410, &K[3][9])
PARAM_ADD(PARAM_FLOAT, k111, &K[0][10])
PARAM_ADD(PARAM_FLOAT, k211, &K[1][10])
PARAM_ADD(PARAM_FLOAT, k311, &K[2][10])
PARAM_ADD(PARAM_FLOAT, k411, &K[3][10])
PARAM_ADD(PARAM_FLOAT, k112, &K[0][11])
PARAM_ADD(PARAM_FLOAT, k212, &K[1][11])
PARAM_ADD(PARAM_FLOAT, k312, &K[2][11])
PARAM_ADD(PARAM_FLOAT, k412, &K[3][11])
PARAM_ADD(PARAM_UINT8, idx, &idx)
PARAM_GROUP_STOP(ctrlLQR)

LOG_GROUP_START(ctrlLQR)
LOG_ADD(LOG_FLOAT, e_x, &state_error_vec[0])
LOG_ADD(LOG_FLOAT, e_y, &state_error_vec[1])
LOG_ADD(LOG_FLOAT, e_z, &state_error_vec[2])
LOG_ADD(LOG_FLOAT, e_roll,  &state_error_vec[3])
LOG_ADD(LOG_FLOAT, e_pitch, &state_error_vec[4])
LOG_ADD(LOG_FLOAT, e_yaw,   &state_error_vec[5])
LOG_ADD(LOG_FLOAT, e_vx, &state_error_vec[6])
LOG_ADD(LOG_FLOAT, e_vy, &state_error_vec[7])
LOG_ADD(LOG_FLOAT, e_vz, &state_error_vec[8])
LOG_ADD(LOG_FLOAT, e_vroll,  &state_error_vec[9])
LOG_ADD(LOG_FLOAT, e_vpitch, &state_error_vec[10])
LOG_ADD(LOG_FLOAT, e_vyaw,   &state_error_vec[11])
LOG_ADD(LOG_FLOAT, u1,   &input_vec[0])
LOG_ADD(LOG_FLOAT, u2,   &input_vec[1])
LOG_ADD(LOG_FLOAT, u3,   &input_vec[2])
LOG_ADD(LOG_FLOAT, u4,   &input_vec[3])
LOG_ADD(LOG_INT16, u2_pwm,   &pwm_int[0])
LOG_ADD(LOG_INT16, u3_pwm,   &pwm_int[1])
LOG_ADD(LOG_INT16, u4_pwm,   &pwm_int[2])
LOG_ADD(LOG_FLOAT, set_pitch, &set_pitch)
LOG_ADD(LOG_FLOAT, thrust, &thrust)
LOG_GROUP_STOP(ctrlLQR)
