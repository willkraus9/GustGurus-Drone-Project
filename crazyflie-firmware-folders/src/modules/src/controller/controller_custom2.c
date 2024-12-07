/*
// __        _____ _   _ ____  ____  ____  _____    _    _  _______ ____  ____  
// \ \      / /_ _| \ | |  _ \| __ )|  _ \| ____|  / \  | |/ / ____|  _ \/ ___| 
//  \ \ /\ / / | ||  \| | | | |  _ \| |_) |  _|   / _ \ | ' /|  _| | |_) \___ \ 
//   \ V  V /  | || |\  | |_| | |_) |  _ <| |___ / ___ \| . \| |___|  _ < ___) |
//    \_/\_/  |___|_| \_|____/|____/|_| \_\_____/_/   \_\_|\_\_____|_| \_\____/ 
//  
//  Crazyflie control firmware
//  
//  Custom implementation of a PID controller via direct thrust/torque calculations by Nikolaj Hindsbo & Denis Alpay
//  
*/

#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
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
#include "controller_custom2.h"
#include <stdbool.h>
#define constrain(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

// PID controllers for each axis

static PID_t x_controller = {.kp = 0.5f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .output_max = 0.3f, .output_min = -0.3f};
static PID_t y_controller = {.kp = 0.5f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .output_max = 0.3f, .output_min = -0.3f};
static PID_t z_controller = {.kp = 0.5f, .ki = 0.0f, .kd = 0.05f, .integral_max = 0.0f, .output_max = 2.0f, .output_min = 0.2f};
static PID_t yaw_controller = {.kp = 0.001f, .ki = 0.0f, .kd = 3e-5f, .integral_max = 0.0f, .output_max = 1.0f, .output_min = -1.0f};
static PID_t roll_controller = {.kp = 0.004f, .ki = 2e-5f, .kd = 0.0007f, .integral_max = 1e-3f, .output_max = 1.0f, .output_min = -1.0f};
static PID_t pitch_controller = {.kp = 0.004f, .ki = 2e-5f, .kd = 0.0007f, .integral_max = 1e-3f, .output_max = 1.0f, .output_min = -1.0f};



// original values (tune as need be)

// Update the PID controller
float DirectThrustPID_update(PID_t *pid, float error, float dt, float baseline, bool verbose) {
    // Integrate the error
    pid->error_integral += error * dt;

    // Constrain the integral term
    if (pid->error_integral > pid->integral_max) pid->error_integral = pid->integral_max;
    if (pid->error_integral < -pid->integral_max) pid->error_integral = -pid->integral_max;

    // Compute the derivative
    float derivative = (error - pid->error_previous) / dt;
    pid->error_previous = error;

    // Compute the PID output
    float output = pid->kp * error + pid->ki * pid->error_integral + pid->kd * derivative + baseline;

    // Constrain the output
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    // DEBUG_PRINT("PID Update - Error: %.3f, Integral: %.3f, Derivative: %.3f, Output: %.3f\n",
    //             (double)error, (double)pid->error_integral, (double)derivative, (double)output);


    return output;
}


void multiply_matrices(const float mat1[3][3], const float mat2[3][3], float result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0; // Initialize the result matrix element to zero
            for (int k = 0; k < 3; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j]; // Perform the multiplication and sum
            }
        }
    }
}

// Function to multiply a 3x3 matrix by a 3x1 vector
Vector3f multiplyMatrixVector(float matrix[3][3], Vector3f vector) {
    Vector3f result;
    result.x = matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z;
    result.y = matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z;
    result.z = matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z;
    return result;
}

// Function to compute the world-to-body frame error
Vector3f World2BodyError(Vector3f error, Vector3f orientation) {
    float roll = orientation.x;
    float pitch = orientation.y;
    float yaw = orientation.z;

    // Compute the rotation matrix from world to body frame
    float Rx[3][3];
    float Ry[3][3];
    float Rz[3][3];
    float cosRoll = cos(-roll), sinRoll = sin(-roll);
    float cosPitch = cos(-pitch), sinPitch = sin(-pitch);
    float cosYaw = cos(-yaw), sinYaw = sin(-yaw);

    Rx[0][0] = 1.0f; Rx[0][1] = 0.0f; Rx[0][2] = 0.0f;
    Rx[1][0] = 0.0f; Rx[1][1] = cosRoll; Rx[1][2] = -sinRoll;
    Rx[2][0] = 0.0f; Rx[2][1] = sinRoll; Rx[2][2] = cosRoll;

    Ry[0][0] = cosPitch; Ry[0][1] = 0.0f; Ry[0][2] = sinPitch;
    Ry[1][0] = 0.0f; Ry[1][1] = 1.0f; Ry[1][2] = 0.0f;
    Ry[2][0] = -sinPitch; Ry[2][1] = 0.0f; Ry[2][2] = cosPitch;

    Rz[0][0] = cosYaw; Rz[0][1] = -sinYaw; Rz[0][2] = 0.0f;
    Rz[1][0] = sinYaw; Rz[1][1] = cosYaw; Rz[1][2] = 0.0f;
    Rz[2][0] = 0.0f; Rz[2][1] = 0.0f; Rz[2][2] = 1.0f;

    float rotation[3][3];
    float temp[3][3];

    multiply_matrices(Rz, Ry, temp);
    multiply_matrices(temp, Rx, rotation);

    // Transform error from world frame to body frame
    return multiplyMatrixVector(rotation, error);
}

// Reset the PID controller
void DirectThrustPID_reset(PID_t *pid) {
    pid->error_integral = 0.0f;
    pid->error_previous = 0.0f;
}

// Initialize custom PID controller
void controllerCustomPidDirectThrustInit() {
    // Initialize all PID controllers
    DEBUG_PRINT("NEW Custom PID Controller Initialized\n");
}

// Test the PID controller
bool controllerCustomPidDirectThrustTest() {
    return true; // Always return true to indicate the controller is functional
}

// Map the setpoint to the desired state
void map_setpoint_to_desired_direct_thrust(const setpoint_t *setpoint, desired_state_t *desired_state) {
    desired_state->x = setpoint->position.x;
    desired_state->y = setpoint->position.y;
    desired_state->z = setpoint->position.z;
    desired_state->yaw = setpoint->attitude.yaw;
}

// Map the current state
void map_state_to_actual_direct_thrust(const state_t *state, current_state_t *current_state) {
    current_state->roll = state->attitude.roll / 57.3f;
    current_state->pitch = state->attitude.pitch / 57.3f;
    current_state->yaw = state->attitude.yaw / 57.3f;
    current_state->z = state->position.z;
    current_state->x = state->position.x;
    current_state->y = state->position.y;
}

// Update the custom PID controller
void controllerCustomPidDirectThrust(
            control_t *control,
            const setpoint_t *setpoint,
            const sensorData_t *sensors,
            const state_t *state,
            const uint32_t stabilizerStep)
{
    if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
        return;
    }
    control->controlMode = controlModeForceTorque;
    static uint32_t lastPrintTime = 0;
    static uint32_t lastPrintTime2 = 0;
    uint32_t currentTime = xTaskGetTickCount();
    float dt = 1.0f / ATTITUDE_RATE;

    desired_state_t desired_state;
    current_state_t current_state;

    map_setpoint_to_desired_direct_thrust(setpoint, &desired_state);
    map_state_to_actual_direct_thrust(state, &current_state);

    if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
        // Compute errors
        bool isToppled = current_state.roll > 1.0f || current_state.roll < -1.0f || current_state.pitch > 1.0f || current_state.pitch < -1.0f;

        float x_error = desired_state.x - (current_state.x);
        float y_error = desired_state.y - (current_state.y);
        float z_error = desired_state.z - (current_state.z);

        // compute body errors
        struct vec error = {x_error, y_error, z_error};
        Vector3f orientation = {current_state.roll, current_state.pitch, current_state.yaw};
        // Vector3f bodyError = World2BodyError(error, orientation);

        struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
        struct mat33 R = quat2rotmat(q);

        struct vec bodyError = mvmul(R, error);

        float x_error_body = bodyError.x;
        float y_error_body = bodyError.y;
        float z_error_body = bodyError.z;

        float pitch_target = DirectThrustPID_update(&x_controller, -x_error_body, dt, 0.0f, false);
        float roll_target = DirectThrustPID_update(&y_controller, -y_error_body, dt, 0.0f, false);

        float roll_error = (roll_target - (current_state.roll));
        float pitch_error =  (pitch_target - (current_state.pitch));

        float thrust = DirectThrustPID_update(&z_controller, z_error, dt, 0.33f, false); // change back to 0.27f for min thrust
        float torqueX = DirectThrustPID_update(&roll_controller, roll_error, dt, 0.0f, false);
        float torqueY = DirectThrustPID_update(&pitch_controller, pitch_error, dt, 0.0f, false);
        float torqueZ = DirectThrustPID_update(&yaw_controller, desired_state.yaw - (current_state.yaw), dt, 0.0f, false);

        // Set the control outputs
        control->thrustSi = thrust;
        control->torqueX = torqueX;
        control->torqueY = -torqueY;
        control->torqueZ = torqueZ;

        // Safety checks
        if (desired_state.z < 0.1f && current_state.z < 0.3f) {
            control->thrustSi = 0.0f;
            control->torqueX = 0.0f;
            control->torqueY = 0.0f;
            control->torqueZ = 0.0f;
        }


        float maxdeg = 1.0f;
        if ((current_state.roll) > maxdeg || (current_state.roll) < -maxdeg ||
            (current_state.pitch) > maxdeg || (current_state.pitch) < -maxdeg) {
            control->thrustSi = 0.0f;
            control->torqueX = 0.0f;
            control->torqueY = 0.0f;
            control->torqueZ = 0.0f;
        }


    //         // Log the control outputs every 500ms
    if (currentTime - lastPrintTime >= pdMS_TO_TICKS(1000) && !isToppled && desired_state.z > 0.1f) {
        DEBUG_PRINT("----------------------------------------------------------------------\n");
        DEBUG_PRINT("Body error - x: %.3f, y: %.3f, z: %.3f\n", (double)x_error_body, (double)y_error_body, (double)z_error_body);
        // DEBUG_PRINT("Global error - x: %.3f, y: %.3f, z: %.3f\n", (double)x_error, (double)y_error, (double)z_error);
        DEBUG_PRINT("Thrust: %.3f, TorqueX: %.6f, TorqueY: %.6f, TorqueZ: %.6f\n", (double)thrust, (double)torqueX, (double)-torqueY, (double)torqueZ);
        DEBUG_PRINT("Roll target: %.3f, Pitch target: %.3f\n", (double)roll_target, (double)pitch_target);
        DEBUG_PRINT("Roll, %.3f, Pitch: %.3f\n", (double)current_state.roll, (double)current_state.pitch);
        DEBUG_PRINT("Integral Roll: %.6f, Integral Pitch: %.6f\n", (double)roll_controller.error_integral, (double)pitch_controller.error_integral);
        
        lastPrintTime = currentTime;
    }

    }

}

#ifdef CRAZYFLIE_FW
// Firmware initialization
void controllerCustomFirmware2Init(void) {
    controllerCustomPidDirectThrustInit();
}

// Firmware test
bool controllerCustomFirmware2Test(void) {
    // always return true
    return true;
}

// Firmware control
void controllerCustomFirmware2(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t stabilizerStep
) {
    controllerCustomPidDirectThrust(control, setpoint, sensors, state, stabilizerStep);
}
#endif

#ifdef CRAZYFLIE_FW
#include "param.h"
PARAM_GROUP_START(ctrlCustom2)

// Roll PID
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_r, &roll_controller.kp)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_r, &roll_controller.ki)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_r, &roll_controller.kd)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, imax_r, &roll_controller.integral_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omax_r, &roll_controller.output_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omin_r, &roll_controller.output_min)

// Pitch PID
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_p, &pitch_controller.kp)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_p, &pitch_controller.ki)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_p, &pitch_controller.kd)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, imax_p, &pitch_controller.integral_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omax_p, &pitch_controller.output_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omin_p, &pitch_controller.output_min)

// Yaw PID
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_y, &yaw_controller.kp)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_y, &yaw_controller.ki)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_y, &yaw_controller.kd)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, imax_y, &yaw_controller.integral_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omax_y, &yaw_controller.output_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omin_y, &yaw_controller.output_min)

// X-axis Thrust PID
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_x, &x_controller.kp)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_x, &x_controller.ki)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_x, &x_controller.kd)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, imax_x, &x_controller.integral_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omax_x, &x_controller.output_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omin_x, &x_controller.output_min)

// Y-axis Thrust PID
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_y_t, &y_controller.kp)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_y_t, &y_controller.ki)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_y_t, &y_controller.kd)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, imax_y_t, &y_controller.integral_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omax_y_t, &y_controller.output_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omin_y_t, &y_controller.output_min)

// Z-axis Thrust PID
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_z, &z_controller.kp)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_z, &z_controller.ki)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_z, &z_controller.kd)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, imax_z, &z_controller.integral_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omax_z, &z_controller.output_max)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, omin_z, &z_controller.output_min)


PARAM_GROUP_STOP(ctrlCustom2)


LOG_GROUP_START(ctrlCustom2)

// LOG_ADD(LOG_FLOAT, KR_x, &g_self.KR.x)
// LOG_ADD(LOG_FLOAT, KR_y, &g_self.KR.y)
// LOG_ADD(LOG_FLOAT, KR_z, &g_self.KR.z)
// LOG_ADD(LOG_FLOAT, Kw_x, &g_self.Komega.x)
// LOG_ADD(LOG_FLOAT, Kw_y, &g_self.Komega.y)
// LOG_ADD(LOG_FLOAT, Kw_z, &g_self.Komega.z)

// LOG_ADD(LOG_FLOAT,Kpos_Px, &g_self.Kpos_P.x)
// LOG_ADD(LOG_FLOAT,Kpos_Py, &g_self.Kpos_P.y)
// LOG_ADD(LOG_FLOAT,Kpos_Pz, &g_self.Kpos_P.z)
// LOG_ADD(LOG_FLOAT,Kpos_Dx, &g_self.Kpos_D.x)
// LOG_ADD(LOG_FLOAT,Kpos_Dy, &g_self.Kpos_D.y)
// LOG_ADD(LOG_FLOAT,Kpos_Dz, &g_self.Kpos_D.z)


// LOG_ADD(LOG_FLOAT, thrustSi, &g_self.thrustSi)
// LOG_ADD(LOG_FLOAT, torquex, &g_self.u.x)
// LOG_ADD(LOG_FLOAT, torquey, &g_self.u.y)
// LOG_ADD(LOG_FLOAT, torquez, &g_self.u.z)

// // current angles
// LOG_ADD(LOG_FLOAT, rpyx, &g_self.rpy.x)
// LOG_ADD(LOG_FLOAT, rpyy, &g_self.rpy.y)
// LOG_ADD(LOG_FLOAT, rpyz, &g_self.rpy.z)

// // desired angles
// LOG_ADD(LOG_FLOAT, rpydx, &g_self.rpy_des.x)
// LOG_ADD(LOG_FLOAT, rpydy, &g_self.rpy_des.y)
// LOG_ADD(LOG_FLOAT, rpydz, &g_self.rpy_des.z)

// // errors
// LOG_ADD(LOG_FLOAT, error_posx, &g_self.p_error.x)
// LOG_ADD(LOG_FLOAT, error_posy, &g_self.p_error.y)
// LOG_ADD(LOG_FLOAT, error_posz, &g_self.p_error.z)

// LOG_ADD(LOG_FLOAT, error_velx, &g_self.v_error.x)
// LOG_ADD(LOG_FLOAT, error_vely, &g_self.v_error.y)
// LOG_ADD(LOG_FLOAT, error_velz, &g_self.v_error.z)

// // omega
// LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
// LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
// LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

// // omega_r
// LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
// LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
// LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

// // LOG_ADD(LOG_UINT32, ticks, &ticks)

LOG_GROUP_STOP(ctrlCustom2)

#endif // CRAZYFLIE_FW defined
