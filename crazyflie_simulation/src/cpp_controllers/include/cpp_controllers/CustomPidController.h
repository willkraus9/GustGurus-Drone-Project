#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "pid.h"
#include "CustomPidControllerTypes.h"


class CustomPidController {
    public:
        pid x_controller;
        pid y_controller;
        pid z_controller;
        pid roll_controller;
        pid pitch_controller;
        pid yaw_controller;

        CustomPidController(pid_gains_t gains);
        CustomPidController();
        ~CustomPidController();
        Eigen::Vector4f update(current_state_t current_state, desired_state_t desired_state, float dt);

        Eigen::Vector3f World2BodyError(Eigen::Vector3f error, Eigen::Vector3f orientation);

        void reset();
};


