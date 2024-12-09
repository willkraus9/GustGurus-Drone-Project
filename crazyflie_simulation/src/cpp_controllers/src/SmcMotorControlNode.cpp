#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <memory>
#include "actuator_msgs/msg/actuators.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "CustomPidController.h"
#include "sliding_mode_control.h"
#include <Eigen/Dense>
#include <cmath>

class SmcMotorControlNode : public rclcpp::Node
{
public:
    current_state_t current_state;
    desired_state_t desired_state;
    pid_gains_t gains;
    CustomPidController controller;

    bool simulation_running;
    float current_simulation_time;
    float past_simulation_time;
    float gamma = 0.005964552; // rotor drag to thrust ratio
    float thrust_coefficient = 1.28192e-08;

    float L = 0.046 * cos(M_PI/4.0); // meters

    Eigen::Matrix4f Thrust_torque_to_motor_forces;
    Eigen::Matrix4f motor_forces_to_thrust_torque;

    params_t params;
    
    SmcMotorControlNode() : Node("smc_motor_control_node")
    {
        // Create a publisher for the motor control topic
        motor_control_pub = this->create_publisher<actuator_msgs::msg::Actuators>("/crazyflie/motor_commands", 10);

        // Create a subscriber for the drone state
        drone_state_sub = this->create_subscription<nav_msgs::msg::Odometry>("crazyflie/odom", 10, std::bind(&SmcMotorControlNode::droneStateCallback, this, std::placeholders::_1));

        // create a subscriber for the simluation time
        simulation_time_sub = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&SmcMotorControlNode::simulationTimeCallback, this, std::placeholders::_1));

        // create subscriber for the goal state
        goal_state_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("crazyflie/goal_state_vector", 10, std::bind(&SmcMotorControlNode::goalStateCallback, this, std::placeholders::_1));

        //
        errors = this->create_publisher<std_msgs::msg::Float32MultiArray>("crazyflie/errors", 10);

        this->declare_parameter<float>("pitch_kp", 2e-5);

        current_state = {
            .x = 0.0,
            .y = 0.0,
            .z = 0.0,
            .roll = 0.0,
            .pitch = 0.0,
            .yaw = 0.0,   
            .d_x = 0.0,
            .d_y = 0.0,
            .d_z = 0.0,
            .d_roll = 0.0,
            .d_pitch = 0.0,
            .d_yaw = 0.0
        };

        desired_state = {
            .x = 0.0,
            .y = 0.0,
            .z = 0.0,
            .yaw = 0.0,
        };


        gains = {
            .kp_x = 0.5,
            .ki_x = 0.0,
            .kd_x = 0.0,
            .integral_max_x = 0.0,
            .output_max_x = 0.2,

            .kp_y = 0.5,
            .ki_y = 0.0,
            .kd_y = 0.0,   
            .integral_max_y = 0.0,
            .output_max_y = 0.2,

            .kp_z = 0.8,
            .ki_z = 0.1,
            .kd_z = 0.3,
            .integral_max_z = 0.30,
            .output_max_z = 0.55,

            .kp_roll = 8.0e-4,
            .ki_roll = 1e-5,
            .kd_roll = 1e-4,
            .integral_max_roll = 0.0,
            .output_max_roll = 1e-2,

            .kp_pitch = 8.0e-4,
            .ki_pitch = 1e-5,
            .kd_pitch = 1e-4,
            .integral_max_pitch = 0.0,
            .output_max_pitch = 1e-2,

            .kp_yaw = 1e-3,
            .ki_yaw = 0.0,
            .kd_yaw = 0.0,
            .integral_max_yaw = 0.0,
            .output_max_yaw = 100.0
        };

        controller = CustomPidController(gains);

        Eigen::Matrix4f scale_matrix;

        scale_matrix << 0.25, 0, 0, 0,
                        0, 0.25/L, 0, 0,
                        0, 0, 0.25/L, 0,
                        0, 0, 0, 0.25/gamma;

        Thrust_torque_to_motor_forces << 1, -1, -1, -1,
                                        1, -1, 1, 1,
                                        1, 1, 1, -1,
                                        1, 1, -1, 1;

        // std::cout << "Thrust_torque_to_motor_forces: " << Thrust_torque_to_motor_forces << std::endl;               

        Thrust_torque_to_motor_forces = Thrust_torque_to_motor_forces * scale_matrix;

        motor_forces_to_thrust_torque << 1.0, 1.0, 1.0, 1.0,
                                        -L, L, L, -L,
                                        L, -L, L, -L,
                                        gamma, gamma, -gamma, -gamma;

        std::cout << "motor_forces_to_thrust_torque: " << motor_forces_to_thrust_torque << std::endl;
        std::cout << "Thrust_torque_to_motor_forces: " << Thrust_torque_to_motor_forces.inverse() << std::endl;


        // std::cout << "Thrust_torque_to_motor_forces: " << Thrust_torque_to_motor_forces << std::endl;
        // std::cout << scale_matrix << std::endl;


        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(2),
                std::bind(&SmcMotorControlNode::timerCallback, this)
            );

  }
private:
    void goalStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Purpose of this function is to receive the goal state at every time step
        desired_state.x = msg->data[0];
        desired_state.y = msg->data[1];
        desired_state.yaw = msg->data[2];
        desired_state.z = msg->data[3];
    }

    void simulationTimeCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
      // Purpose of this function is to receive the simulation time at every time step
      double nanos = msg->clock.nanosec;
      int secs = msg->clock.sec;

      current_simulation_time = secs * 1e9 + nanos;
      // conver time to milliseconds
      current_simulation_time = current_simulation_time / 1e6;

      // Check if the simulation is running
      if (current_simulation_time > past_simulation_time){
            simulation_running = true;
      } // Check if the simulation is reset
        else if (current_simulation_time < past_simulation_time || current_simulation_time == 0.0){
            simulation_running = false;
            controller.reset();
      } // Check if the simulation is paused
        else if (current_simulation_time == past_simulation_time){
            simulation_running = false;
      }
        past_simulation_time = current_simulation_time;
    }

    void droneStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Purpose of this function is to receive the state of the drone at every time step

        float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;
        float z = msg->pose.pose.position.z;
        float quat_x = msg->pose.pose.orientation.x;
        float quat_y = msg->pose.pose.orientation.y;
        float quat_z = msg->pose.pose.orientation.z;
        float quat_w = msg->pose.pose.orientation.w;

        // Convert quaternion to euler angles
        std::vector<float> euler = Quat2Euler(quat_w, quat_x, quat_y, quat_z);
        double roll, pitch, yaw;
        
        roll = euler[0];
        pitch = euler[1];
        yaw = euler[2];

        // Linear velocities derivatives
        float d_x = msg->twist.twist.linear.x;
        float d_y = msg->twist.twist.linear.y;
        float d_z = msg->twist.twist.linear.z;

        // Angular velocities derivatives
        float d_roll = msg->twist.twist.angular.x;
        float d_pitch = msg->twist.twist.angular.y;
        float d_yaw = msg->twist.twist.angular.z;

        // std::cout << "Roll: " << roll << " Pitch: " << pitch << std::endl;

        // float roll_d = msg->twist.twist.angular.x;
        current_state = {
            .x = x,
            .y = y,
            .z = z,
            .roll = roll,
            .pitch = pitch,
            .yaw = yaw,
            .d_x = d_x,
            .d_y = d_y,
            .d_z = d_z,
            .d_roll = d_roll,
            .d_pitch = d_pitch, 
            .d_yaw = d_yaw
        };
    }

    void timerCallback()
    {
      // Purpose of thsi function is to publish motor speed to the gazebo drone at every time step
      // TODO: Implement the PID controller here
      // The PID controller should calculate the motor speeds based on the current state of the drone
        // Call the PID fixed height controller to compute control commands

        // if the simulation is not running, do not publish motor commands
        if (!simulation_running){
          return;
        }

        float pitch_kp;
        // this->get_parameter("pitch_kp", pitch_kp);

        // gains.kp_pitch = pitch_kp;
        // gains.kp_roll = pitch_kp;

        double dt = 0.001; // 1ms timer interval

        float error_x = desired_state.x - current_state.x;
        float error_y = desired_state.y - current_state.y;
        float error_z = desired_state.z - current_state.z;
        Eigen::Vector3f error = {error_x, error_y, error_z};
        Eigen::Vector3f current_orientation(current_state.roll, current_state.pitch, current_state.yaw);

        Eigen::Vector3f body_error = controller.World2BodyError(error, current_orientation);

        float roll_target = controller.x_controller.update(body_error[0], dt, 0.0);
        float pitch_target = controller.y_controller.update(body_error[1], dt, 0.0);

        float error_d_z = 0.0 - current_state.d_z;
        float error_roll = roll_target - current_state.roll;
        float error_d_roll = 0.0 - current_state.d_roll;
        float error_pitch = pitch_target - current_state.pitch;
        float error_d_pitch = 0.0 - current_state.d_pitch;

        Eigen::Vector4f thrust_torque = controller.update(current_state, desired_state, dt);

        // Create and populate the actuator message
        
        // std::cout << "Thrust_torque: " << thrust_torque << std::endl;
        float thrust = smc_z(error_z, error_d_z, current_state.roll, current_state.pitch, params);
        float roll = smc_roll(error_roll, current_state.d_pitch, current_state.d_yaw, error_d_roll, params);
        float pitch = smc_pitch(error_pitch, current_state.d_roll, current_state.d_yaw, error_d_pitch, params);
        

        thrust_torque[0] = thrust;
        thrust_torque[1] = roll;
        thrust_torque[2] = pitch;
        RCLCPP_INFO(this->get_logger(), "Thrust_torque: %f, %f, %f, %f", thrust_torque[0], thrust_torque[1], thrust_torque[2], thrust_torque[3]);
        Eigen::Vector4f motor_forces = Thrust_torque_to_motor_forces * thrust_torque;
        // std::cout << "Motor forces: " << motor_forces << std::endl;

        for (int i = 0; i < 4; i++){
          if (motor_forces[i] < 0){
            motor_forces[i] = 0;
          }
        }

        Eigen::Vector4f motor_speeds = sqrt(motor_forces.array() / thrust_coefficient);

        auto msg = actuator_msgs::msg::Actuators();
        msg.velocity = {motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]};
        motor_control_pub->publish(msg);

        // pid_attitude_fixed_height_controller2(state, &goal, gain, dt, &motor_commands);
        // Eigen::Vector4d motor_speeds = {motor_commands.m1, motor_commands.m2, motor_commands.m3, motor_commands.m4};
        // Eigen::Vector4d motor_forces = thrust_coefficient * motor_speeds.array() * motor_speeds.array();
        // Eigen::Vector4d Thrust_torque = motor_forces_to_thrust_torque * motor_forces;
        // std::cout << "Thrust_torque: " << Thrust_torque << std::endl;
        // auto msg = actuator_msgs::msg::Actuators();
        // msg.velocity = {motor_commands.m1, motor_commands.m2, motor_commands.m3, motor_commands.m4};
        // motor_control_pub->publish(msg);


        // RCLCPP_INFO(this->get_logger(), "Publishing motor speeds: %f", speed);
    }

    float WrapAngle(float angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
    }

    std::vector<float> Quat2Euler(float w, float x, float y, float z) {
        // Wrap angle to [-pi, pi]
        float sinr_cosp = 2.0 * (w * x + y * z);
        float cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        float roll = std::atan2(sinr_cosp, cosr_cosp);

        // Calculate pitch (y-axis rotation)
        float sinp = 2.0 * (w * y - z * x);
        float pitch;
        if (std::abs(sinp) >= 1.0)
            pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // Calculate yaw (z-axis rotation)
        float siny_cosp = 2.0 * (w * z + x * y);
        float cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        float yaw = std::atan2(siny_cosp, cosy_cosp);

        // Wrap angles to [-pi, pi]
        roll = WrapAngle(roll);
        pitch = WrapAngle(pitch);
        yaw = WrapAngle(yaw);

        // Return Euler angles as a vector
        return {roll, pitch, yaw};  
    }
    
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_control_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_state_sub;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr simulation_time_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr goal_state_sub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr errors;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmcMotorControlNode>());
  rclcpp::shutdown();
  return 0;
};