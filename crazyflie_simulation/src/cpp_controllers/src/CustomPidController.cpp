#include "CustomPidController.h"

CustomPidController::CustomPidController(pid_gains_t gains) {
    x_controller = pid(gains.kp_x, gains.ki_x, gains.kd_x, gains.integral_max_x, gains.output_max_x);
    y_controller = pid(gains.kp_y, gains.ki_y, gains.kd_y, gains.integral_max_y, gains.output_max_y);
    z_controller = pid(gains.kp_z, gains.ki_z, gains.kd_z, gains.integral_max_z, gains.output_max_z);
    roll_controller = pid(gains.kp_roll, gains.ki_roll, gains.kd_roll, gains.integral_max_roll, gains.output_max_roll);
    pitch_controller = pid(gains.kp_pitch, gains.ki_pitch, gains.kd_pitch, gains.integral_max_pitch, gains.output_max_pitch);
    yaw_controller = pid(gains.kp_yaw, gains.ki_yaw, gains.kd_yaw, gains.integral_max_yaw, gains.output_max_yaw);
}

CustomPidController::CustomPidController(){}

CustomPidController::~CustomPidController() {}


Eigen::Vector4f CustomPidController::update(current_state_t current_state, desired_state_t desired_state, float dt) {
    // Calculate errors
    float x_error = desired_state.x - current_state.x;
    float y_error = desired_state.y - current_state.y;
    float z_error = desired_state.z - current_state.z;


    Eigen::Vector3f world_pos_error(x_error, y_error, z_error);
    Eigen::Vector3f current_orientation(current_state.roll, current_state.pitch, current_state.yaw);

    Eigen::Vector3f body_pos_error = World2BodyError(world_pos_error, current_orientation);

    float body_x_error = body_pos_error(0);
    float body_y_error = body_pos_error(1);
    float body_z_error = body_pos_error(2);

    // Update controllers
    float roll_target = x_controller.update(body_x_error, dt, 0.0);
    float pitch_target = y_controller.update(body_y_error, dt, 0.0);


    float roll_error = roll_target - current_state.roll;
    float pitch_error = pitch_target - current_state.pitch;

    std::cout << "body z error: " << body_z_error << std::endl;
    std::cout << "roll_error: " << roll_error << std::endl;
    std::cout << "pitch_error: " << pitch_error << std::endl;


    float thrust = z_controller.update(body_z_error, dt, 0.27468, false);
    float torqueX = roll_controller.update(roll_error, dt, 0.0, false);
    float torqueY = pitch_controller.update(pitch_error, dt, 0.0);
    float torqueZ = yaw_controller.update(desired_state.yaw - current_state.yaw, dt, 0.0);
    
    return Eigen::Vector4f(thrust, torqueX, torqueY, torqueZ);
}

Eigen::Vector3f CustomPidController::World2BodyError(Eigen::Vector3f error, Eigen::Vector3f orientation)
{
    float roll = orientation(0);
    float pitch = orientation(1);
    float yaw = orientation(2);

    // Compute the rotation matrix from world to body frame using explicit matrices
    Eigen::Matrix3f rotation;

    // Roll (rotation about X-axis)
    Eigen::Matrix3f rollMatrix;
    rollMatrix << 1, 0, 0,
                  0, cos(roll), -sin(roll),
                  0, sin(roll), cos(roll);

    // Pitch (rotation about Y-axis)
    Eigen::Matrix3f pitchMatrix;
    pitchMatrix << cos(pitch), 0, -sin(pitch),
                   0, 1, 0,
                   sin(pitch), 0, cos(pitch);

    // Yaw (rotation about Z-axis)
    Eigen::Matrix3f yawMatrix;
    yawMatrix << cos(yaw), -sin(yaw), 0,
                 sin(yaw), cos(yaw), 0,
                 0, 0, 1;

    // Combine the rotations to get the world-to-body rotation matrix
    rotation = yawMatrix * pitchMatrix * rollMatrix;

    // Transform error from world frame to body frame
    return rotation * error;
}


void CustomPidController::reset() {
    x_controller.error_integral = 0;
    y_controller.error_integral = 0;
    z_controller.error_integral = 0;
    roll_controller.error_integral = 0;
    pitch_controller.error_integral = 0;
    yaw_controller.error_integral = 0;

    x_controller.error_previous = 0;
    y_controller.error_previous = 0;
    z_controller.error_previous = 0;
    roll_controller.error_previous = 0;
    pitch_controller.error_previous = 0;
    yaw_controller.error_previous = 0;

}

